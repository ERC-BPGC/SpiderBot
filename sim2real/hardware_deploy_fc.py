"""
Script to deploy ONNX policy on hardware, but with fixed velocity commands (Vx, Vy, Vz) provided via command line arguments.
Inherits logic from hardware_deploy.py.
"""

import argparse
import time

import numpy as np
import onnxruntime as ort
from hardware_deploy import (
  ACTION_SCALE,
  CONTROL_DT,
  ONNX_MODEL_PATH,
  KeyboardController,
  build_observation,
  move_to_home_position,
  portHandler_sc,
  portHandler_st,
  publish,
  read_servo_positions,
  scale_action_to_servo_ticks,
  servo_ticks_to_radians,
  setup_ports,
)


def main():
  parser = argparse.ArgumentParser(
    description="Deploy ONNX policy with fixed velocity commands."
  )
  parser.add_argument("--vx", type=float, default=0.0, help="Linear velocity x (m/s)")
  parser.add_argument("--vy", type=float, default=0.0, help="Linear velocity y (m/s)")
  parser.add_argument(
    "--vz", type=float, default=0.0, help="Angular velocity z (rad/s)"
  )
  args = parser.parse_args()

  command = np.array([args.vx, args.vy, args.vz], dtype=np.float32)
  print(f"Using fixed command: vx={args.vx}, vy={args.vy}, vz={args.vz}")

  print(f"Loading ONNX model from {ONNX_MODEL_PATH} ...")
  ort_session = ort.InferenceSession(ONNX_MODEL_PATH)
  input_meta = ort_session.get_inputs()[0]
  raw_obs_dim = input_meta.shape[-1]
  expected_obs_dim = int(raw_obs_dim) if str(raw_obs_dim).isdigit() else 27
  use_joint_vel = expected_obs_dim == 39
  print(
    f"  Expected obs dim : {expected_obs_dim} ({'+ joint_vel' if use_joint_vel else 'no joint_vel'})"
  )

  setup_ports()
  move_to_home_position()
  print("Hardware initialized.\n")

  last_action = np.zeros(12, dtype=np.float32)
  prev_joint_pos_rad = np.zeros(12, dtype=np.float32)
  kb_controller = KeyboardController()

  print("--- Hexapod ONNX Hardware Controller (Fixed Command) ---")
  print("Press 'q' to stop or Ctrl+C.")

  try:
    while True:
      kb_controller.poll()
      if kb_controller.is_pressed("q"):
        print("Quit requested.")
        break

      loop_start = time.time()
      positions_ticks = read_servo_positions()
      joint_pos_rad = servo_ticks_to_radians(positions_ticks)
      joint_vel_rad = (joint_pos_rad - prev_joint_pos_rad) / CONTROL_DT
      prev_joint_pos_rad = joint_pos_rad.copy()

      observation = build_observation(
        joint_pos_rad,
        last_action,
        command,
        joint_vel_rad=joint_vel_rad if use_joint_vel else None,
      )

      obs_batch = observation.reshape(1, -1)
      ort_inputs = {input_meta.name: obs_batch}
      ort_outputs = ort_session.run(None, ort_inputs)
      assert isinstance(ort_outputs[0], np.ndarray), "Expected dense policy actions"
      action = ort_outputs[0][0]
      action = action * ACTION_SCALE
      action = np.clip(action, -1.0, 1.0)

      robot_command_ticks = scale_action_to_servo_ticks(action)
      publish(robot_command_ticks)
      last_action = action.copy().astype(np.float32)

      elapsed = time.time() - loop_start
      sleep_time = max(0.0, CONTROL_DT - elapsed)
      time.sleep(sleep_time)

  except KeyboardInterrupt:
    print("\nKeyboard interrupt received.")

  print("\nStopping robot and cleaning up resources...")
  kb_controller.stop()
  move_to_home_position()
  time.sleep(1)
  portHandler_sc.closePort()
  portHandler_st.closePort()
  print("Cleanup complete.")


if __name__ == "__main__":
  main()
