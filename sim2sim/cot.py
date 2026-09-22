"""
Sim2Sim inference script for Hexapod policy with CoT calculation.
Runs robot still for N steps, then commands forward velocity, then stops and reports CoT.
"""

import argparse
import os
import time
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
from history_obs import (
  ObservationHistory,
  build_actor_observation,
  process_policy_action,
)
from mujoco import viewer

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
XML_PATH = os.environ.get(
  "SPIDERBOT_XML_PATH", str(SCRIPT_DIR / "xmls" / "flat_terrain.xml")
)
# Override with a policy exported for this script's observation/action configuration.
ONNX_MODEL_PATH = os.environ.get(
  "SPIDERBOT_POLICY_PATH", str(SCRIPT_DIR / "policies" / "spiderbot.onnx")
)

CONTROL_FREQ = 40  # Hz: 0.005s physics timestep * mjlab decimation 5
ACTION_SCALE = 0.25
RAW_ACTION_CLIP = 2.0
DEFAULT_HISTORY_LENGTH = 3

ROBOT_MASS_KG = 3.0  # kg — update if different
COMMAND_VEL_X = 0.25  # m/s forward command during locomotion phase

# Experiment phases (in control steps at 50 Hz)
STILL_STEPS = 100  # 2 s standing still
LOCOMOTION_STEPS = 500  # 10 s walking
# Script exits after STILL_STEPS + LOCOMOTION_STEPS

# Actuated joint names in the same natural order used by MJLab action targets:
# calf, parallel-top for each leg.
ACTUATED_JOINT_NAMES = [
  "calf_motor_link_joint_leg_1",
  "parallel_link_top_joint_leg_1",
  "calf_motor_link_joint_leg_2",
  "parallel_link_top_joint_leg_2",
  "calf_motor_link_joint_leg_3",
  "parallel_link_top_joint_leg_3",
  "calf_motor_link_joint_leg_4",
  "parallel_link_top_joint_leg_4",
  "calf_motor_link_joint_leg_5",
  "parallel_link_top_joint_leg_5",
  "calf_motor_link_joint_leg_6",
  "parallel_link_top_joint_leg_6",
]
PROCESSED_ACTION_CLIP = (
  np.array([-0.65, -0.35] * 6, dtype=np.float32),
  np.array([0.65, 0.55] * 6, dtype=np.float32),
)

# ============================================================================
# Helper Functions
# ============================================================================


def get_sensor_data(model, data, sensor_name):
  sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
  sensor_adr = model.sensor_adr[sensor_id]
  sensor_dim = model.sensor_dim[sensor_id]
  return data.sensordata[sensor_adr : sensor_adr + sensor_dim].copy()


def get_projected_gravity(model, data):
  upvector = get_sensor_data(model, data, "upvector")
  return -1 * upvector / (np.linalg.norm(upvector) + 1e-8)


def get_joint_indices(model, joint_names):
  qpos_indices, qvel_indices = [], []
  for joint_name in joint_names:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    joint_type = model.jnt_type[joint_id]
    if joint_type != mujoco.mjtJoint.mjJNT_HINGE:
      raise ValueError(f"Unsupported joint type for {joint_name}")
    qpos_indices.append(model.jnt_qposadr[joint_id])
    qvel_indices.append(model.jnt_dofadr[joint_id])
  return np.array(qpos_indices), np.array(qvel_indices)


def get_actuator_indices(model, joint_names):
  actuator_indices = []
  for joint_name in joint_names:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    found = False
    for act_id in range(model.nu):
      if (
        model.actuator_trntype[act_id] == mujoco.mjtTrn.mjTRN_JOINT
        and model.actuator_trnid[act_id, 0] == joint_id
      ):
        actuator_indices.append(act_id)
        found = True
        break
    if not found:
      raise ValueError(f"No actuator found for joint {joint_name}")
  return np.array(actuator_indices)


def compute_instantaneous_power(data, qvel_indices, actuator_indices):
  """
  Mechanical power = sum over joints of (torque * angular_velocity).
  Uses data.actuator_force for torques and qvel for joint velocities.
  Only counts positive power (no recuperation assumed, matching paper assumption).
  """
  torques = data.actuator_force[actuator_indices]  # Nm
  print(f"DEBUG: torques = {torques}")
  ang_vels = data.qvel[qvel_indices]  # rad/s
  joint_powers = torques * ang_vels  # W per joint
  # Sum all joint powers (positive = consuming, negative = regenerating)
  # For a non-recuperating system, clamp negatives to 0
  return float(np.sum(np.maximum(joint_powers, 0.0)))


# ============================================================================
# Main
# ============================================================================


def parse_args():
  parser = argparse.ArgumentParser(description="Run Spiderbot sim2sim CoT test.")
  parser.add_argument(
    "--history-length",
    type=int,
    default=DEFAULT_HISTORY_LENGTH,
    help="Number of actor observation frames for non-command terms. Use 1 for old single-frame policies.",
  )
  return parser.parse_args()


def main():
  args = parse_args()

  print(f"Loading MuJoCo model from {XML_PATH}...")
  model = mujoco.MjModel.from_xml_path(XML_PATH)
  data = mujoco.MjData(model)

  print(f"Loading ONNX model from {ONNX_MODEL_PATH}...")
  ort_session = ort.InferenceSession(ONNX_MODEL_PATH)
  input_shape = ort_session.get_inputs()[0].shape
  print(f"ONNX input shape: {input_shape}")
  print(f"Observation history length: {args.history_length}")

  qpos_indices, qvel_indices = get_joint_indices(model, ACTUATED_JOINT_NAMES)
  actuator_indices = get_actuator_indices(model, ACTUATED_JOINT_NAMES)

  mujoco.mj_resetData(model, data)
  mujoco.mj_forward(model, data)
  default_joint_pos = data.qpos[qpos_indices].copy()

  dt_control = 1.0 / CONTROL_FREQ

  last_action = np.zeros(len(ACTUATED_JOINT_NAMES), dtype=np.float32)
  obs_history = ObservationHistory(args.history_length)

  # ── CoT tracking (locomotion phase only) ──────────────────────────────
  loco_energy_J = 0.0  # accumulated mechanical energy during locomotion
  loco_distance_m = 0.0  # accumulated path length during locomotion
  loco_start_pos = None  # XY position at start of locomotion phase (for reference only)
  prev_loco_pos = None  # XY position at previous control step

  total_steps = STILL_STEPS + LOCOMOTION_STEPS
  step_count = 0

  print(
    f"\nPhase 1: Standing still for {STILL_STEPS} steps ({STILL_STEPS / CONTROL_FREQ:.1f} s)"
  )
  print(
    f"Phase 2: Walking at {COMMAND_VEL_X} m/s for {LOCOMOTION_STEPS} steps ({LOCOMOTION_STEPS / CONTROL_FREQ:.1f} s)"
  )
  print(f"Total steps: {total_steps}\n")

  with viewer.launch_passive(model, data) as gui:
    last_control_time = time.time()

    while gui.is_running() and step_count < total_steps:
      current_time = time.time()

      if current_time - last_control_time >= dt_control:
        # ── Phase control ──────────────────────────────────────────
        in_locomotion = step_count >= STILL_STEPS

        if step_count == STILL_STEPS:
          loco_start_pos = data.qpos[:2].copy()
          prev_loco_pos = loco_start_pos.copy()
          print(
            f"[Step {step_count}] Locomotion phase started. "
            f"Start pos: ({loco_start_pos[0]:.3f}, {loco_start_pos[1]:.3f})"
          )

        command = np.array(
          [COMMAND_VEL_X, 0.0, 0.0] if in_locomotion else [0.0, 0.0, 0.0],
          dtype=np.float32,
        )

        # ── Observation ───────────────────────────────────────────
        base_ang_vel = get_sensor_data(model, data, "imu_ang_vel")
        projected_gravity = get_projected_gravity(model, data)
        joint_pos = data.qpos[qpos_indices] - default_joint_pos
        joint_vel = data.qvel[qvel_indices] * 0.05

        observation = build_actor_observation(
          obs_history,
          base_ang_vel,
          projected_gravity,
          joint_pos,
          joint_vel,
          last_action,
          command,
        )

        # ── Policy inference ──────────────────────────────────────
        ort_inputs = {ort_session.get_inputs()[0].name: observation.reshape(1, -1)}
        output = ort_session.run(None, ort_inputs)[0]
        assert isinstance(output, np.ndarray), "Expected dense policy actions"
        raw_action = output[0]
        last_action, action = process_policy_action(
          raw_action,
          ACTION_SCALE,
          raw_clip=RAW_ACTION_CLIP,
          processed_clip=PROCESSED_ACTION_CLIP,
        )

        # ── Apply actions ─────────────────────────────────────────
        data.ctrl[actuator_indices] = default_joint_pos + action

        # ── Energy + distance accumulation (locomotion phase only) ─
        if in_locomotion:
          power = compute_instantaneous_power(data, qvel_indices, actuator_indices)
          loco_energy_J += power * dt_control  # J = W * s

          current_pos = data.qpos[:2].copy()
          loco_distance_m += np.linalg.norm(current_pos - prev_loco_pos)
          prev_loco_pos = current_pos

        # ── Logging ───────────────────────────────────────────────
        if step_count % 50 == 0:
          phase = "WALKING" if in_locomotion else "STILL"
          power_now = compute_instantaneous_power(data, qvel_indices, actuator_indices)
          print(
            f"[{phase}] Step {step_count:4d} | "
            f"cmd: {command[0]:+.2f} m/s | "
            f"power: {power_now:6.2f} W | "
            f"dist: {loco_distance_m:.3f} m | "
            f"pos: ({data.qpos[0]:.3f}, {data.qpos[1]:.3f})"
          )

        step_count += 1
        last_control_time = current_time

      mujoco.mj_step(model, data)
      gui.sync()
      time.sleep(0.005)

    loco_end_pos = data.qpos[:2].copy()

  # ── CoT Report ────────────────────────────────────────────────────────
  loco_end_pos = data.qpos[:2].copy()
  displacement = np.linalg.norm(
    loco_end_pos - loco_start_pos
  )  # straight-line, for reference
  g = 9.81

  if loco_distance_m < 0.01:
    print("\n⚠️  Warning: distance travelled is near zero — robot may not have moved.")
    print("   CoT calculation may be unreliable.\n")

  cot = (
    loco_energy_J / (ROBOT_MASS_KG * g * loco_distance_m)
    if loco_distance_m > 0.01
    else float("nan")
  )

  loco_duration_s = LOCOMOTION_STEPS / CONTROL_FREQ
  avg_power_W = loco_energy_J / loco_duration_s
  avg_speed = loco_distance_m / loco_duration_s

  print("\n" + "=" * 55)
  print("  CoT RESULTS (locomotion phase only)")
  print("=" * 55)
  print(f"  Robot mass:          {ROBOT_MASS_KG:.2f} kg")
  print(f"  Commanded velocity:  {COMMAND_VEL_X:.2f} m/s")
  print(f"  Avg speed (path):    {avg_speed:.3f} m/s")
  print(f"  Path length:         {loco_distance_m:.3f} m  (accumulated)")
  print(f"  Displacement:        {displacement:.3f} m  (straight-line, for ref)")
  print(f"  Locomotion duration: {loco_duration_s:.1f} s")
  print(f"  Total mech. energy:  {loco_energy_J:.2f} J")
  print(f"  Average mech. power: {avg_power_W:.2f} W")
  print(f"  CoT (mech. only):    {cot:.4f}")
  print("=" * 55)
  print()
  print("  Note: This CoT counts only mechanical joint power")
  print("  (τ·ω, positive only). It excludes electronics,")
  print("  compute, and servo driver losses. For total CoT,")
  print("  add your measured standby power draw.")
  if not np.isnan(cot):
    standby_W = 1.50  # rough Jetson Nano estimate — adjust as needed
    total_energy = loco_energy_J + standby_W * loco_duration_s
    cot_total = total_energy / (ROBOT_MASS_KG * g * loco_distance_m)
    print(
      f"\n  Estimated total CoT (assuming ~{standby_W:.0f}W standby): {cot_total:.4f}"
    )
  print("=" * 55)


if __name__ == "__main__":
  main()
