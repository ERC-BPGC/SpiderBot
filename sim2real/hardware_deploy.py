"""
Hardware deployment script for Hexapod using ONNX policy.
Select the exported policy with SPIDERBOT_POLICY_PATH.
No mjlab/brax/jax required. Uses onnxruntime, numpy, and the bundled servo SDK.
"""

import csv
import importlib
import os
import sys
import time

import numpy as np
import onnxruntime as ort

# --- Hardware Imports ---
SERVO_SDK_PATH = os.environ.get(
  "SPIDERBOT_SERVO_SDK_PATH", os.path.dirname(os.path.abspath(__file__))
)
sys.path.insert(0, SERVO_SDK_PATH)
# Use the bundled SDK unless an external SDK directory is selected.
servo_sdk = importlib.import_module("scservo_sdk")
PortHandler = servo_sdk.PortHandler
scscl = servo_sdk.scscl
sms_sts = servo_sdk.sms_sts

# --- Non-blocking keyboard input (no X server / display required) ---
# Works over SSH and on headless Linux boards.
if os.name == "nt":
  import msvcrt

  def _getch() -> str | None:
    """Return one character if a key is waiting, else None."""
    if msvcrt.kbhit():
      return msvcrt.getch().decode("utf-8", errors="ignore")
    return None

  def _restore_terminal() -> None:
    pass  # nothing to restore on Windows

else:
  import select as _select
  import termios
  import tty

  _stdin_fd = sys.stdin.fileno()
  _orig_term = termios.tcgetattr(_stdin_fd)
  # Switch to raw mode once so individual reads are instant
  tty.setraw(_stdin_fd)

  def _getch() -> str | None:
    """Return one character if a key is waiting, else None."""
    if _select.select([sys.stdin], [], [], 0)[0]:
      return sys.stdin.read(1)
    return None

  def _restore_terminal() -> None:
    """Restore the terminal to its original cooked mode."""
    try:
      termios.tcsetattr(_stdin_fd, termios.TCSADRAIN, _orig_term)
    except Exception:
      pass


# ==================================================================
# PATHS (relative to this script's directory)
# ==================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ONNX_MODEL_PATH = os.environ.get("SPIDERBOT_POLICY_PATH", "")
# Select a compatible no-IMU policy explicitly; the original t8.onnx is not bundled.

# ==================================================================
# HARDWARE & ROBOT CONFIGURATION
# ==================================================================

# --- Serial Port Configuration ---
BAUDRATE = 1000000
DEVICENAME_sc = os.environ.get("SPIDERBOT_SC_PORT", "/dev/ttyACM0")
DEVICENAME_st = os.environ.get("SPIDERBOT_ST_PORT", "/dev/ttyACM1")

# Initialize Port and Packet Handlers
portHandler_sc = PortHandler(DEVICENAME_sc)  # noqa: F405
portHandler_st = PortHandler(DEVICENAME_st)  # noqa: F405
packetHandler_sc = scscl(portHandler_sc)  # noqa: F405
packetHandler_st = sms_sts(portHandler_st)  # noqa: F405

# --- Servo Configuration (CRITICAL: TUNE THESE VALUES) ---
# Maps logical leg index (0-5) to physical servo ID.
ID_SC = [6, 3, 1, 5, 2, 4]  # Hip servos (SC type)
ID_ST = [7, 8, 9, 10, 11, 12]  # Calf servos (ST type)

# Servo middle/home positions (in ticks).
MIDDLE_POS_SC = {1: 433, 2: 526, 3: 493, 4: 555, 5: 491, 6: 482}
MIDDLE_POS_ST = {7: 1970, 8: 3059, 9: 2029, 10: 1846, 11: 2039, 12: 2056}

# --- Scaling & Safety Limits ---
# Must match the action_scale used during AI training.
ACTION_SCALE = 0.5

# Conversion from radians (AI output) to servo ticks.
RADIANS_TO_TICKS_SC = 300
RADIANS_TO_TICKS_ST = 750

# SAFETY: Symmetric ± tick offsets from each servo's MIDDLE_POS.
# Limits are computed at runtime as  MIDDLE_POS ± offset,
# so changing MIDDLE_POS automatically shifts both limits.
SC_LIMIT_NEG = 175  # ticks below MIDDLE_POS_SC  (hip range backward)
SC_LIMIT_POS = 175  # ticks above MIDDLE_POS_SC  (hip range forward)
ST_LIMIT_NEG = 400  # ticks below MIDDLE_POS_ST  (calf range down)
ST_LIMIT_POS = 400  # ticks above MIDDLE_POS_ST  (calf range up)

# --- Control Loop ---
CONTROL_FREQ = 50  # Hz (match training)
CONTROL_DT = 1.0 / CONTROL_FREQ


# ==================================================================
# HELPER CLASSES
# ==================================================================


class KeyboardController:
  """
  Non-blocking keyboard controller using raw terminal I/O.
  Works headless (no X server / display needed).

  Behaviour: each call to poll() reads the latest available character.
  The most-recently pressed key stays "active" until a new key is pressed
  or clear() is called.  is_pressed(k) is True when k is the active key.
  """

  def __init__(self) -> None:
    self._active_key: str | None = None

  def poll(self) -> str | None:
    """Read latest keypress (if any) and update active key."""
    ch = _getch()
    if ch is not None:
      self._active_key = ch
    return ch

  def is_pressed(self, key_char: str) -> bool:
    return self._active_key == key_char

  def clear(self) -> None:
    """Forget the active key (e.g. after stop or quit is handled)."""
    self._active_key = None

  def stop(self) -> None:
    _restore_terminal()


class VelocityCommandController:
  """
  Velocity command controller with acceleration ramping and decay.

  Holding a directional key ramps speed up toward the cap.
  Releasing a key lets velocity decay to zero.
  Pressing 's' instantly zeroes all velocities (stop / home).
  Pressing 'q' signals quit.

  Key layout:
      w       – forward (ramp +yvel)
      x       – backward (ramp -yvel)
      a       – strafe left (ramp -xvel)
      d       – strafe right (ramp +xvel)
      r       – rotate right (ramp +yaw)
      e       – rotate left (ramp -yaw)
      s       – STOP: zero all velocities + home
      q       – quit
  """

  MAX_LIN_VEL: float = 0.5  # m/s cap
  MAX_ANG_VEL: float = 1.0  # rad/s cap
  LIN_ACCEL: float = 0.008  # m/s added per control tick while key held
  ANG_ACCEL: float = 0.04  # rad/s added per control tick while key held
  DECAY: float = 0.80  # multiplicative decay per tick when key not held
  DEADZONE: float = 0.005  # snap to zero below this magnitude

  def __init__(self, kb: KeyboardController) -> None:
    self.kb = kb
    self.xvel: float = 0.0
    self.yvel: float = 0.0
    self.yaw: float = 0.0
    self.stop_requested: bool = False
    self.quit_requested: bool = False

  def reset(self) -> None:
    """Zero all velocities (called on 's' and at shutdown)."""
    self.xvel = 0.0
    self.yvel = 0.0
    self.yaw = 0.0

  def update(self) -> np.ndarray:
    """
    Call once per control tick.
    Returns the current command as np.ndarray([xvel, yvel, yaw]).
    Sets self.stop_requested / self.quit_requested as side-effects.
    """
    kb = self.kb

    self.quit_requested = kb.is_pressed("q")

    # Stop: zero velocities, signal caller to home the robot
    if kb.is_pressed("s"):
      if not self.stop_requested:  # edge: only print once per press
        print("COMMAND: STOP")
      self.reset()
      self.stop_requested = True
      return np.zeros(3, dtype=np.float32)

    self.stop_requested = False

    # --- Forward / Backward (w / x) ---
    if kb.is_pressed("w"):
      self.yvel = min(self.yvel + self.LIN_ACCEL, self.MAX_LIN_VEL)
    elif kb.is_pressed("x"):
      self.yvel = max(self.yvel - self.LIN_ACCEL, -self.MAX_LIN_VEL)
    else:
      self.yvel *= self.DECAY
      if abs(self.yvel) < self.DEADZONE:
        self.yvel = 0.0

    # --- Strafe left / right (a / d) ---
    if kb.is_pressed("a"):
      self.xvel = max(self.xvel - self.LIN_ACCEL, -self.MAX_LIN_VEL)
    elif kb.is_pressed("d"):
      self.xvel = min(self.xvel + self.LIN_ACCEL, self.MAX_LIN_VEL)
    else:
      self.xvel *= self.DECAY
      if abs(self.xvel) < self.DEADZONE:
        self.xvel = 0.0

    # --- Yaw right / left (r / e) ---
    if kb.is_pressed("r"):
      self.yaw = min(self.yaw + self.ANG_ACCEL, self.MAX_ANG_VEL)
    elif kb.is_pressed("e"):
      self.yaw = max(self.yaw - self.ANG_ACCEL, -self.MAX_ANG_VEL)
    else:
      self.yaw *= self.DECAY
      if abs(self.yaw) < self.DEADZONE:
        self.yaw = 0.0

    return np.array([self.xvel, self.yvel, self.yaw], dtype=np.float32)


# ==================================================================
# HARDWARE CONTROL FUNCTIONS
# ==================================================================


def setup_ports():
  """Opens and configures the serial ports for the servos."""
  if not portHandler_sc.openPort():
    print("Failed to open SC servo port")
    quit()
  if not portHandler_sc.setBaudRate(BAUDRATE):
    print("Failed to change SC baudrate")
    quit()
  print("SC servo port opened successfully.")

  if not portHandler_st.openPort():
    print("Failed to open ST servo port")
    quit()
  if not portHandler_st.setBaudRate(BAUDRATE):
    print("Failed to change ST baudrate")
    quit()
  print("ST servo port opened successfully.")


def publish(command_ticks: np.ndarray):
  """
  Sends a 12-element command array (in ticks) to the hexapod's motors.
  Indices 0-5 → SC servos (IDs 1-6), indices 6-11 → ST servos (IDs 7-12).
  """
  command = np.round(command_ticks).astype(int)

  for scs_id in range(1, 7):
    motor_pos_sc = int(command[scs_id - 1])
    packetHandler_sc.SyncWritePos(scs_id, motor_pos_sc, 0, 3000)

  for sts_id in range(7, 13):
    motor_pos_st = int(command[sts_id - 1])
    packetHandler_st.SyncWritePosEx(sts_id, motor_pos_st, 5000, 100)

  packetHandler_sc.groupSyncWrite.txPacket()
  packetHandler_st.groupSyncWrite.txPacket()

  packetHandler_sc.groupSyncWrite.clearParam()
  packetHandler_st.groupSyncWrite.clearParam()


def move_to_home_position():
  """Moves the robot to its neutral standing position."""
  print("Moving to home position...")
  home_command = np.zeros(12)
  for i in range(1, 7):
    home_command[i - 1] = MIDDLE_POS_SC[i]
  for i in range(7, 13):
    home_command[i - 1] = MIDDLE_POS_ST[i]
  publish(home_command)
  time.sleep(1.5)


def read_servo_positions() -> np.ndarray:
  """
  Reads the current positions of all servos.
  Returns a 12-element array interleaved as [SC_leg0, ST_leg0, SC_leg1, ...].
  """
  positions = np.zeros(12)
  for i in range(6):
    scs_id = ID_SC[i]
    sts_id = ID_ST[i]
    pos_sc, _, _ = packetHandler_sc.ReadPos(scs_id)
    pos_st, _, _ = packetHandler_st.ReadPos(sts_id)
    positions[i * 2] = pos_sc
    positions[i * 2 + 1] = pos_st
  return positions


# ==================================================================
# SIM-TO-REAL TRANSFORMATION
# ==================================================================


def servo_ticks_to_radians(positions: np.ndarray) -> np.ndarray:
  """
  Convert raw servo tick positions to joint angles in radians
  (relative to default / home position).
  Returns 12-element array interleaved [hip0, calf0, hip1, calf1, ...].
  """
  joint_angles = np.zeros(12)
  for i in range(6):
    # Hip (SC)
    scs_id = ID_SC[i]
    joint_angles[i * 2] = (
      positions[i * 2] - MIDDLE_POS_SC[scs_id]
    ) / RADIANS_TO_TICKS_SC
    # Calf (ST)
    sts_id = ID_ST[i]
    joint_angles[i * 2 + 1] = (
      positions[i * 2 + 1] - MIDDLE_POS_ST[sts_id]
    ) / RADIANS_TO_TICKS_ST
  return joint_angles


def scale_action_to_servo_ticks(ai_action: np.ndarray) -> np.ndarray:
  """
  Converts the AI's action (radians, relative to default) into absolute
  servo tick commands with safety clipping.
  """
  robot_command_ticks = np.zeros(12)

  for i in range(6):
    # --- Hip (SC) Servo ---
    scs_id = ID_SC[i]
    hip_delta_rad = ai_action[i * 2]
    hip_delta_ticks = hip_delta_rad * RADIANS_TO_TICKS_SC
    hip_target_ticks = MIDDLE_POS_SC[scs_id] + hip_delta_ticks
    robot_command_ticks[scs_id - 1] = np.clip(
      hip_target_ticks,
      MIDDLE_POS_SC[scs_id] - SC_LIMIT_NEG,
      MIDDLE_POS_SC[scs_id] + SC_LIMIT_POS,
    )

    # --- Calf (ST) Servo ---
    sts_id = ID_ST[i]
    calf_delta_rad = ai_action[i * 2 + 1]
    calf_delta_ticks = calf_delta_rad * RADIANS_TO_TICKS_ST
    calf_target_ticks = MIDDLE_POS_ST[sts_id] + calf_delta_ticks
    robot_command_ticks[sts_id - 1] = np.clip(
      calf_target_ticks,
      MIDDLE_POS_ST[sts_id] - ST_LIMIT_NEG,
      MIDDLE_POS_ST[sts_id] + ST_LIMIT_POS,
    )

  return robot_command_ticks


def build_observation(
  joint_pos_rad: np.ndarray,
  last_action: np.ndarray,
  command: np.ndarray,
  joint_vel_rad: np.ndarray | None = None,
) -> np.ndarray:
  """
  Build the observation vector for the policy.

  Layout (27-dim, no joint velocities):
      joint_pos(12)  – current joint angles relative to default (rad)
      last_action(12) – previous policy output
      command(3)     – [lin_vel_x, lin_vel_y, ang_vel_z]

  Layout (39-dim, with joint velocities):
      joint_pos(12)  – current joint angles relative to default (rad)
      joint_vel(12)  – joint angular velocities (rad/s), finite-differenced
      last_action(12) – previous policy output
      command(3)     – [lin_vel_x, lin_vel_y, ang_vel_z]

  joint_vel_rad: if provided (non-None), the 39-dim layout is used.
  """
  if joint_vel_rad is not None:
    observation = np.concatenate(
      [
        joint_pos_rad,  # 12
        joint_vel_rad,  # 12
        last_action,  # 12
        command,  # 3
      ]
    ).astype(np.float32)
  else:
    observation = np.concatenate(
      [
        joint_pos_rad,  # 12
        last_action,  # 12
        command,  # 3
      ]
    ).astype(np.float32)
  return observation


# ==================================================================
# MAIN
# ==================================================================


def main():
  # ------------------------------------------------------------------
  # 1. Load ONNX policy
  # ------------------------------------------------------------------
  print(f"Loading ONNX model from {ONNX_MODEL_PATH} ...")
  ort_session = ort.InferenceSession(ONNX_MODEL_PATH)
  input_meta = ort_session.get_inputs()[0]
  print(f"  Input name : {input_meta.name}")
  print(f"  Input shape: {input_meta.shape}")
  print(f"  Input dtype: {input_meta.type}")
  output_meta = ort_session.get_outputs()[0]
  print(f"  Output name : {output_meta.name}")
  print(f"  Output shape: {output_meta.shape}")
  print("ONNX model loaded successfully.\n")

  # Determine expected observation size from the ONNX model's input shape.
  # Shape is typically [batch, obs_dim]; handle both symbolic and numeric dims.
  raw_obs_dim = input_meta.shape[-1]
  expected_obs_dim = int(raw_obs_dim) if str(raw_obs_dim).isdigit() else 27
  use_joint_vel = expected_obs_dim == 39
  print(
    f"  Expected obs dim : {expected_obs_dim} "
    f"({'+ joint_vel' if use_joint_vel else 'no joint_vel'})"
  )

  # ------------------------------------------------------------------
  # 2. Initialize hardware
  # ------------------------------------------------------------------
  setup_ports()
  move_to_home_position()
  print("Hardware initialized.\n")

  # ------------------------------------------------------------------
  # 3. Initialize controllers and state
  # ------------------------------------------------------------------
  kb_controller = KeyboardController()
  vel_controller = VelocityCommandController(kb_controller)
  last_action = np.zeros(12, dtype=np.float32)
  command = np.zeros(3, dtype=np.float32)

  prev_joint_pos_rad = np.zeros(12, dtype=np.float32)  # for velocity estimation

  print("--- Hexapod ONNX Hardware Controller Ready ---")
  print("Controls:")
  print("  w       = Forward         (hold to accelerate)")
  print("  x       = Backward        (hold to accelerate)")
  print("  a / d   = Strafe L / R    (hold to accelerate)")
  print("  r / e   = Rotate R / L    (hold to accelerate)")
  print("  s       = STOP + Home position")
  print("  q       = Quit")
  print()

  # CSV logging
  num_actions = 12
  header = [f"action_{i + 1}" for i in range(num_actions)]

  log_path = os.path.join(SCRIPT_DIR, "actions_log_hardware.csv")
  with open(log_path, "w", newline="") as f:
    csv_writer = csv.writer(f)
    csv_writer.writerow(header)

    step = 0
    prev_time = time.time()

    try:
      while True:
        loop_start = time.time()

        # ------------------------------------------------------
        # 3a. Keyboard commands (ramping velocity controller)
        # ------------------------------------------------------
        kb_controller.poll()  # read latest keypress into active key
        command = vel_controller.update()

        if vel_controller.quit_requested:
          print("Quit requested.")
          break

        if vel_controller.stop_requested:
          # Reset policy state and send robot home
          last_action = np.zeros(12, dtype=np.float32)
          prev_joint_pos_rad = np.zeros(12, dtype=np.float32)
          kb_controller.clear()  # don't re-trigger stop every tick
          move_to_home_position()
          step = 0
          loop_start = time.time()
          continue

        # ------------------------------------------------------
        # 3b. Read joint positions (ticks → radians)
        # ------------------------------------------------------
        # Reading every loop can be slow; switch to
        # `joint_pos_rad = last_action.copy()` if latency is too high.
        positions_ticks = read_servo_positions()
        joint_pos_rad = servo_ticks_to_radians(positions_ticks)

        # Compute joint velocities via finite difference (rad/s)
        joint_vel_rad = (joint_pos_rad - prev_joint_pos_rad) / CONTROL_DT
        prev_joint_pos_rad = joint_pos_rad.copy()

        # ------------------------------------------------------
        # 3c. Build observation (joint_vel included only if model
        #     expects 39-dim input)
        # ------------------------------------------------------
        observation = build_observation(
          joint_pos_rad,
          last_action,
          command,
          joint_vel_rad=joint_vel_rad if use_joint_vel else None,
        )

        # ------------------------------------------------------
        # 3d. Run ONNX inference
        # ------------------------------------------------------
        obs_batch = observation.reshape(1, -1)
        ort_inputs = {input_meta.name: obs_batch}
        ort_outputs = ort_session.run(None, ort_inputs)
        assert isinstance(ort_outputs[0], np.ndarray), "Expected dense policy actions"
        action = ort_outputs[0][0]  # remove batch dim

        # Scale & clip
        action = action * ACTION_SCALE
        action = np.clip(action, -1.0, 1.0)

        # Log
        csv_writer.writerow(action.tolist())

        # ------------------------------------------------------
        # 3e. Convert action → servo ticks & publish
        # ------------------------------------------------------
        robot_command_ticks = scale_action_to_servo_ticks(action)
        publish(robot_command_ticks)

        # ------------------------------------------------------
        # 3f. Update state
        # ------------------------------------------------------
        last_action = action.copy().astype(np.float32)
        step += 1

        # ------------------------------------------------------
        # 3g. Debug print (every ~1 s)
        # ------------------------------------------------------
        now = time.time()
        if step % CONTROL_FREQ == 0:
          hz = 1.0 / (now - prev_time + 1e-9)
          print(
            f"Step {step:5d} | "
            f"cmd=[{command[0]:+.2f},{command[1]:+.2f},{command[2]:+.2f}] | "
            f"act mean={action.mean():.3f} std={action.std():.3f} | "
            f"Hz={hz:.1f}"
          )
        prev_time = now

        # ------------------------------------------------------
        # 3h. Rate-limit to CONTROL_FREQ
        # ------------------------------------------------------
        elapsed = time.time() - loop_start
        sleep_time = max(0.0, CONTROL_DT - elapsed)
        time.sleep(sleep_time)

    except KeyboardInterrupt:
      print("\nKeyboard interrupt received.")

  # ------------------------------------------------------------------
  # 4. Cleanup
  # ------------------------------------------------------------------
  print("\nStopping robot and cleaning up resources...")
  move_to_home_position()
  time.sleep(1)
  kb_controller.stop()
  portHandler_sc.closePort()
  portHandler_st.closePort()
  print("Cleanup complete.")


if __name__ == "__main__":
  main()
