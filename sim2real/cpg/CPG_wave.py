import importlib
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

# --- Non-blocking keyboard input setup (from your original code) ---
if os.name == "nt":
  import msvcrt

  def getch():
    if msvcrt.kbhit():
      return msvcrt.getch().decode()
    else:
      return None
else:
  import select
  import sys
  import termios
  import tty

  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)

  def getch():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
      try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
      finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
      return ch
    else:
      return None


SDK_ROOT = os.environ.get(
  "SPIDERBOT_SERVO_SDK_PATH", str(Path(__file__).resolve().parents[1])
)
sys.path.insert(0, SDK_ROOT)
servo_sdk = importlib.import_module("scservo_sdk")
PortHandler = servo_sdk.PortHandler
scscl = servo_sdk.scscl
sms_sts = servo_sdk.sms_sts

# --- Servo SDK and Port Configuration (from your original code) ---
BAUDRATE = 1000000
DEVICENAME_sc = os.environ.get("SPIDERBOT_SC_PORT", "/dev/ttyACM0")
DEVICENAME_st = os.environ.get("SPIDERBOT_ST_PORT", "/dev/ttyACM1")

portHandler_sc = PortHandler(DEVICENAME_sc)
portHandler_st = PortHandler(DEVICENAME_st)
packetHandler_sc = scscl(portHandler_sc)
packetHandler_st = sms_sts(portHandler_st)

# --- Robot Geometry & Kinematics ---
LEG_ANGLES_DEG = np.array([60.0, 0.0, -60.0, -120.0, 180.0, 120.0])
LEG_ANGLES_RAD = np.deg2rad(LEG_ANGLES_DEG)
LEG_RADIUS = 150.0
MAX_VERTICAL_LIFT = 60.0

# --- Gait Parameters (TUNE THESE) ---
STRIDE_LENGTH = 225.0
STEP_HEIGHT = 70.0
GAIT_SPEED_HZ = 0.70  # Wave gait is naturally slower, you might adjust this
TURN_RATE_DEG = 35.0

# --- Servo Configuration ---
ID_SC = [6, 3, 1, 5, 2, 4]
ID_ST = [7, 8, 9, 10, 11, 12]
MIDDLE_POS_SC = {
  1: 443,  # Leg 2
  2: 536,  # Leg 4
  3: 485,  # Leg 1
  4: 580,  # Leg 5
  5: 431,  # Leg 3
  6: 438,  # Leg 0
}
MIDDLE_POS_ST = {
  7: 1987,  # Leg 0
  8: 2038,  # Leg 1
  9: 2050,  # Leg 2
  10: 1827,  # Leg 3
  11: 2017,  # Leg 4
  12: 2050,  # Leg 5
}
HIP_DEG_TO_POS = 5.0
CALF_LIFT_TO_POS = 10.0

# --- Wave Gait Configuration (NEW SECTION) ---
# This list defines the order in which the legs will lift and move.
# A common stable sequence: Right-Back, Right-Mid, Right-Front,
# then Left-Back, Left-Mid, Left-Front.
# Logical leg indices are:
# Leg 0: Front-Right  Leg 1: Front-Left
# Leg 2: Mid-Right    Leg 3: Mid-Left
# Leg 4: Back-Right   Leg 5: Back-Left
WAVE_SEQUENCE = [4, 2, 0, 5, 3, 1]

# Calculate the phase offset for each leg based on its position in the sequence.
# The total cycle (2*pi) is divided by the number of legs.
LEG_PHASE_OFFSETS = np.zeros(6)
for i, leg_index in enumerate(WAVE_SEQUENCE):
  LEG_PHASE_OFFSETS[leg_index] = (i * (2.0 * math.pi)) / 6.0


def setup_ports():
  """Opens and configures the serial ports for the servos."""
  if portHandler_sc.openPort():
    print("Succeeded to open the SC servo port")
  else:
    print("Failed to open the SC servo port")
    quit()
  if portHandler_st.openPort():
    print("Succeeded to open the ST servo port")
  else:
    print("Failed to open the ST servo port")
    quit()
  if portHandler_sc.setBaudRate(BAUDRATE):
    print("Succeeded to change the SC baudrate")
  else:
    print("Failed to change the SC baudrate")
    quit()
  if portHandler_st.setBaudRate(BAUDRATE):
    print("Succeeded to change the ST baudrate")
  else:
    print("Failed to change the ST baudrate")
    quit()


def publish(command):
  """Sends a 12-element command array to the hexapod's motors."""
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


def calculate_ik(target_x, target_y):
  """Calculates the hip and calf servo angles for a given leg target."""
  x_component = np.clip(target_x, -LEG_RADIUS, LEG_RADIUS)
  hip_angle_deg = math.asin(x_component / LEG_RADIUS) * 180.0 / math.pi
  calf_angle_deg = -np.clip(target_y, 0, MAX_VERTICAL_LIFT)
  return hip_angle_deg, calf_angle_deg


def move_to_home_position():
  """Moves the robot to its neutral standing position."""
  print("Moving to home position...")
  hip_angle_deg, calf_angle_deg = calculate_ik(0, 0)
  robot_command = np.zeros(12)
  for i in range(6):
    scs_id, sts_id = ID_SC[i], ID_ST[i]
    hip_pos = MIDDLE_POS_SC[scs_id] + (hip_angle_deg * HIP_DEG_TO_POS)
    calf_pos = MIDDLE_POS_ST[sts_id] + (calf_angle_deg * CALF_LIFT_TO_POS)
    robot_command[scs_id - 1], robot_command[sts_id - 1] = hip_pos, calf_pos
  publish(robot_command)
  time.sleep(1)


def main():
  setup_ports()
  walking, turn_rate_rad, phase = False, 0.0, 0.0
  last_update_time = time.time()
  move_to_home_position()

  print("\n--- Python CPG Hexapod Controller (Wave Gait) ---")
  print("STATUS: STOPPED")
  print("Controls: w=Walk, a=Left, d=Right, s=Stop, q=Quit")

  try:
    while True:
      char = getch()
      if char:
        should_start_walking = not walking
        if char == "q":
          break
        elif char == "w":
          print("COMMAND: Walk Straight")
          turn_rate_rad = 0.0
          if should_start_walking:
            walking = True
        elif char == "a":
          print("COMMAND: Turn Left")
          turn_rate_rad = np.deg2rad(TURN_RATE_DEG)
          if should_start_walking:
            walking = True
        elif char == "d":
          print("COMMAND: Turn Right")
          turn_rate_rad = -np.deg2rad(TURN_RATE_DEG)
          if should_start_walking:
            walking = True
        elif char == "s":
          print("COMMAND: Stop")
          if walking:
            walking = False
            move_to_home_position()

        if should_start_walking and walking:
          print("STATUS: WALKING STARTED")
          phase, last_update_time = 0.0, time.time()

      if not walking:
        time.sleep(0.02)
        continue

      # 1. Update Master Clock
      current_time = time.time()
      time_delta = current_time - last_update_time
      last_update_time = current_time
      phase_increment = time_delta * GAIT_SPEED_HZ * 2.0 * math.pi
      phase = (phase + phase_increment) % (2.0 * math.pi)

      # --- CPG Execution Block (MODIFIED FOR WAVE GAIT) ---
      robot_command = np.zeros(12)
      for i in range(6):  # For each leg from 0 to 5
        # 2. Calculate this leg's unique phase
        leg_phase = (phase + LEG_PHASE_OFFSETS[i]) % (2.0 * math.pi)

        # 3. Generate rhythm and lift signals from the leg's unique phase
        rhythm_signal = math.sin(leg_phase)
        body_y_target = max(0.0, math.cos(leg_phase)) * STEP_HEIGHT

        # Trajectory Generation
        body_x_target = -rhythm_signal * (STRIDE_LENGTH / 2.0)

        # Add Turning Motion and Geometric Correction
        turn_x = -turn_rate_rad * LEG_RADIUS * math.sin(LEG_ANGLES_RAD[i])
        turn_z = turn_rate_rad * LEG_RADIUS * math.cos(LEG_ANGLES_RAD[i])
        leg_x_target = (body_x_target + turn_x) * math.cos(LEG_ANGLES_RAD[i]) + (
          turn_z
        ) * math.sin(LEG_ANGLES_RAD[i])
        leg_y_target = body_y_target

        # Inverse Kinematics
        hip_angle, calf_angle = calculate_ik(leg_x_target, leg_y_target)

        # Servo Command Generation
        scs_id, sts_id = ID_SC[i], ID_ST[i]
        hip_pos = MIDDLE_POS_SC[scs_id] + (hip_angle * HIP_DEG_TO_POS)
        calf_pos = MIDDLE_POS_ST[sts_id] + (calf_angle * CALF_LIFT_TO_POS)
        hip_pos = np.clip(
          hip_pos, MIDDLE_POS_SC[scs_id] - 175, MIDDLE_POS_SC[scs_id] + 175
        )
        calf_pos = np.clip(calf_pos, MIDDLE_POS_ST[sts_id] - 400, MIDDLE_POS_ST[sts_id])
        robot_command[scs_id - 1] = hip_pos
        robot_command[sts_id - 1] = calf_pos

      # 4. Publish to hardware
      publish(robot_command)
      time.sleep(0.015)

  finally:
    print("\nStopping robot and closing ports...")
    if walking:
      move_to_home_position()
      time.sleep(0.5)
    portHandler_sc.closePort()
    portHandler_st.closePort()
    if os.name != "nt":
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
  main()
