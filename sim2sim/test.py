"""
Sim2Sim inference script for Hexapod policy.
Loads an ONNX model and runs it in pure MuJoCo.
"""

import argparse
import mujoco
from mujoco import viewer
import numpy as np
import time
import onnxruntime as ort
from collections import deque

from history_obs import (
    ObservationHistory,
    build_actor_observation,
    process_policy_action,
)

# ============================================================================
# Configuration
# ============================================================================

XML_PATH = "/home/marmot/Ritwik/mjlab_spiderbot/sim2sim/xmls/flat_terrain.xml"
ONNX_MODEL_PATH = "/home/marmot/Ritwik/mjlab_spiderbot/logs/rsl_rl/spiderbot_velocity/2026-06-10_23-59-58/2026-06-10_23-59-58.onnx"  # Path to your exported ONNX model

CONTROL_FREQ = 50  # Hz (policy runs at 50 Hz)
ACTION_SCALE = 0.25  # Spiderbot action scale from MJLab env config.
RAW_ACTION_CLIP = 2.0
DEFAULT_HISTORY_LENGTH = 3

# Actuated joint names in the same natural order used by MJLab action targets:
# calf, parallel-top for each leg.
ACTUATED_JOINT_NAMES = [
    "calf_motor_link_joint_leg_1", "parallel_link_top_joint_leg_1",
    "calf_motor_link_joint_leg_2", "parallel_link_top_joint_leg_2",
    "calf_motor_link_joint_leg_3", "parallel_link_top_joint_leg_3",
    "calf_motor_link_joint_leg_4", "parallel_link_top_joint_leg_4",
    "calf_motor_link_joint_leg_5", "parallel_link_top_joint_leg_5",
    "calf_motor_link_joint_leg_6", "parallel_link_top_joint_leg_6",
]
PROCESSED_ACTION_CLIP = (
    np.array([-0.65, -0.35] * 6, dtype=np.float32),
    np.array([0.65, 0.55] * 6, dtype=np.float32),
)

# Command limits (for keyboard control)
MAX_LIN_VEL_X = 0.5  # m/s
MAX_LIN_VEL_Y = 0.5  # m/s
MAX_ANG_VEL_Z = 0.7  # rad/s

# ============================================================================
# Helper Functions
# ============================================================================

def get_sensor_data(model, data, sensor_name):
    """Get sensor data by name."""
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    sensor_adr = model.sensor_adr[sensor_id]
    sensor_dim = model.sensor_dim[sensor_id]
    return data.sensordata[sensor_adr:sensor_adr + sensor_dim].copy()


def get_projected_gravity(model, data):
    """
    Compute projected gravity vector in body frame.
    This is the z-axis of the world frame projected into the body frame.
    """
    # Get the upvector (z-axis in body frame)
    upvector = get_sensor_data(model, data, "upvector")
    # Normalize
    upvector = -1 * upvector / (np.linalg.norm(upvector) + 1e-8)
    return upvector


def get_joint_indices(model, joint_names):
    """Get joint qpos and qvel indices for given joint names."""
    qpos_indices = []
    qvel_indices = []
    
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        joint_type = model.jnt_type[joint_id]
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_dofadr = model.jnt_dofadr[joint_id]
        
        # For hinge joints (type 3), qpos and qvel are 1D
        if joint_type == mujoco.mjtJoint.mjJNT_HINGE:
            qpos_indices.append(joint_qposadr)
            qvel_indices.append(joint_dofadr)
        else:
            raise ValueError(f"Unsupported joint type for {joint_name}")
    
    return np.array(qpos_indices), np.array(qvel_indices)


def get_actuator_indices(model, joint_names):
    """Get actuator ctrl indices for given joint names."""
    actuator_indices = []
    
    for joint_name in joint_names:
        # Find actuator that controls this joint
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        
        # Search through actuators to find one that controls this joint
        found = False
        for act_id in range(model.nu):
            # Check if this actuator's transmission targets this joint
            if model.actuator_trntype[act_id] == mujoco.mjtTrn.mjTRN_JOINT:
                trnid = model.actuator_trnid[act_id, 0]
                if trnid == joint_id:
                    actuator_indices.append(act_id)
                    found = True
                    break
        
        if not found:
            raise ValueError(f"No actuator found for joint {joint_name}")
    
    return np.array(actuator_indices)


class KeyboardCommandInterface:
    """Handle keyboard input for velocity commands."""
    
    def __init__(self, max_lin_x, max_lin_y, max_ang_z):
        self.max_lin_x = max_lin_x
        self.max_lin_y = max_lin_y
        self.max_ang_z = max_ang_z
        
        # Current command
        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel_z = 0.0
        
        # Increment per key press
        self.increment = 0.1
        self.ang_increment = 0.1
        
    def update(self, key_presses):
        """
        Update commands based on key presses.
        key_presses: dict of {key: bool} indicating which keys are pressed
        """
        # W/S for forward/backward
        if key_presses.get('w', False):
            self.lin_vel_x = min(self.lin_vel_x + self.increment, self.max_lin_x)
        if key_presses.get('s', False):
            self.lin_vel_x = max(self.lin_vel_x - self.increment, -self.max_lin_x)
        
        # A/D for left/right
        if key_presses.get('a', False):
            self.lin_vel_y = min(self.lin_vel_y + self.increment, self.max_lin_y)
        if key_presses.get('d', False):
            self.lin_vel_y = max(self.lin_vel_y - self.increment, -self.max_lin_y)
        
        # O/P for yaw left/right
        if key_presses.get('o', False):
            self.ang_vel_z = min(self.ang_vel_z + self.ang_increment, self.max_ang_z)
        if key_presses.get('p', False):
            self.ang_vel_z = max(self.ang_vel_z - self.ang_increment, -self.max_ang_z)
        
        # Space to reset commands
        if key_presses.get('space', False):
            self.lin_vel_x = 0.0
            self.lin_vel_y = 0.0
            self.ang_vel_z = 0.0
        
        # Decay towards zero when no input
        decay_rate = 0.95
        if not any([key_presses.get(k, False) for k in ['w', 's']]):
            self.lin_vel_x *= decay_rate
        if not any([key_presses.get(k, False) for k in ['a', 'd']]):
            self.lin_vel_y *= decay_rate
        if not any([key_presses.get(k, False) for k in ['o', 'p']]):
            self.ang_vel_z *= decay_rate
    
    def get_command(self):
        """Return current command as [lin_vel_x, lin_vel_y, ang_vel_z]."""
        return np.array([self.lin_vel_x, self.lin_vel_y, self.ang_vel_z], dtype=np.float32)


# ============================================================================
# Main Inference Loop
# ============================================================================

def parse_args():
    parser = argparse.ArgumentParser(description="Run Spiderbot sim2sim policy playback.")
    parser.add_argument(
        "--history-length",
        type=int,
        default=DEFAULT_HISTORY_LENGTH,
        help="Number of actor observation frames for non-command terms. Use 1 for old single-frame policies.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Load MuJoCo model
    print(f"Loading MuJoCo model from {XML_PATH}...")
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    
    # Load ONNX model
    print(f"Loading ONNX model from {ONNX_MODEL_PATH}...")
    ort_session = ort.InferenceSession(ONNX_MODEL_PATH)
    input_shape = ort_session.get_inputs()[0].shape
    print(f"ONNX input shape: {input_shape}")
    print(f"Observation history length: {args.history_length}")
    
    # Get joint and actuator indices
    qpos_indices, qvel_indices = get_joint_indices(model, ACTUATED_JOINT_NAMES)
    actuator_indices = get_actuator_indices(model, ACTUATED_JOINT_NAMES)
    
    print(f"Number of actuated joints: {len(ACTUATED_JOINT_NAMES)}")
    print(f"Actuator indices: {actuator_indices}")
    print("\nJoint to Actuator Mapping:")
    for i, (joint_name, act_idx) in enumerate(zip(ACTUATED_JOINT_NAMES, actuator_indices)):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_idx)
        print(f"  [{i:2d}] {joint_name:35s} -> actuator[{act_idx}] = {actuator_name}")
    
    # Get default joint positions (initial state)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    default_joint_pos = data.qpos[qpos_indices].copy()
    
    print(f"Default joint positions: {default_joint_pos}")
    
    # Initialize command interface
    cmd_interface = KeyboardCommandInterface(MAX_LIN_VEL_X, MAX_LIN_VEL_Y, MAX_ANG_VEL_Z)
    
    # Action history (for observation)
    last_action = np.zeros(len(ACTUATED_JOINT_NAMES), dtype=np.float32)
    obs_history = ObservationHistory(args.history_length)
    
    # Control timing
    dt_control = 1.0 / CONTROL_FREQ
    dt_sim = model.opt.timestep
    n_sim_steps = int(dt_control / dt_sim)
    
    print(f"\nControl frequency: {CONTROL_FREQ} Hz")
    print(f"Simulation timestep: {dt_sim*1000:.1f} ms")
    print(f"Simulation steps per control step: {n_sim_steps}")
    print(f"\nControls:")
    print("  W/S: Forward/Backward")
    print("  A/D: Left/Right")
    print("  O/P: Rotate Left/Right")
    print("  SPACE: Stop")
    print("  ESC: Quit\n")
    
    # Key press tracking - use persistent state instead of per-frame
    keys_held = {
        'w': False, 's': False, 'a': False, 'd': False,
        'o': False, 'p': False, 'space': False
    }
    
    def key_callback(keycode):
        """Handle key presses - this doesn't work well with passive viewer."""
        # Note: viewer.launch_passive doesn't support key callbacks properly
        # We'll use a different approach
        pass
    
    # Launch viewer
    with viewer.launch_passive(model, data) as gui:
        print("✅ Viewer launched. Starting policy inference...\n")
        print("=" * 60)
        print("MANUAL COMMAND MODE")
        print("=" * 60)
        print("The MuJoCo viewer doesn't support keyboard input well.")
        print("Commands will be set manually in the code.")
        print("\nTo test different commands, modify the script and set:")
        print("  cmd_interface.lin_vel_x = 0.3  # Forward")
        print("  cmd_interface.ang_vel_z = 0.5  # Turn left")
        print("=" * 60)
        print()
        
        # MANUAL COMMAND SETTING - Change these values to test different behaviors
        # After step 100, command forward motion
        TEST_FORWARD_AFTER_STEPS = 100
        
        step_count = 0
        last_control_time = time.time()
        
        while gui.is_running():
            current_time = time.time()
            
            # Control at specified frequency
            if current_time - last_control_time >= dt_control:
                
                # MANUAL COMMAND: Set commands here for testing
                if step_count == TEST_FORWARD_AFTER_STEPS:
                    print(f"\n{'='*60}")
                    print(f"ACTIVATING FORWARD COMMAND at step {step_count}")
                    print(f"{'='*60}\n")
                    cmd_interface.lin_vel_x = 0.5  # Move forward
                    # cmd_interface.ang_vel_z = 0.25  # Uncomment to test turning
                
                # You can also add step-based command changes:
                # if step_count == 300:
                #     cmd_interface.lin_vel_x = 0.0
                #     cmd_interface.ang_vel_z = 0.5  # Start turning
                
                # ================================================================
                # 1. Construct Observation
                # ================================================================
                
                # Base angular velocity (from gyro)
                base_ang_vel = get_sensor_data(model, data, "imu_ang_vel")
                
                # Projected gravity
                projected_gravity = get_projected_gravity(model, data)
                
                # Joint positions (relative to default)
                joint_pos = data.qpos[qpos_indices] - default_joint_pos
                
                # Joint velocities
                joint_vel = data.qvel[qvel_indices] * 0.05
                
                # Last action
                actions_obs = last_action.copy()
                
                # Command (from manual setting above)
                command = cmd_interface.get_command()
                
                # Order:
                # base_ang_vel history + projected_gravity history + joint_pos history
                # + joint_vel history + actions history + single-frame command.
                observation = build_actor_observation(
                    obs_history,
                    base_ang_vel,
                    projected_gravity,
                    joint_pos,
                    joint_vel,
                    actions_obs,
                    command,
                )
                
                # ================================================================
                # 2. Run Policy Inference
                # ================================================================
                
                # ONNX expects batch dimension
                obs_batch = observation.reshape(1, -1)
                
                # Run inference
                ort_inputs = {ort_session.get_inputs()[0].name: obs_batch}
                ort_outputs = ort_session.run(None, ort_inputs)
                raw_action = ort_outputs[0][0]  # Remove batch dimension
                last_action, action = process_policy_action(
                    raw_action,
                    ACTION_SCALE,
                    raw_clip=RAW_ACTION_CLIP,
                    processed_clip=PROCESSED_ACTION_CLIP,
                )
                
                # ================================================================
                # 3. Apply Actions to Actuators
                # ================================================================
                
                # Actions are position targets relative to default
                # Scale actions to joint ranges
                target_positions = default_joint_pos + action
                
                # Set actuator controls (position actuators)
                data.ctrl[actuator_indices] = target_positions
                
                # ================================================================
                # 4. Print Debug Info
                # ================================================================
                
                if step_count % 50 == 0:  # Print every second at 50Hz
                    print(f"Step {step_count}")
                    print(f"  Command: [{command[0]:+.2f}, {command[1]:+.2f}, {command[2]:+.2f}]")
                    print(f"  Base ang vel: [{base_ang_vel[0]:+.3f}, {base_ang_vel[1]:+.3f}, {base_ang_vel[2]:+.3f}]")
                    print(f"  Raw action mean: {last_action.mean():.3f}, std: {last_action.std():.3f}, range: [{last_action.min():.3f}, {last_action.max():.3f}]")
                    print(f"  Joint delta mean: {action.mean():.3f}, std: {action.std():.3f}, range: [{action.min():.3f}, {action.max():.3f}]")
                    print(f"  Base pos (xyz): [{data.qpos[0]:.3f}, {data.qpos[1]:.3f}, {data.qpos[2]:.3f}]")
                    print()
                
                step_count += 1
                last_control_time = current_time
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Sync viewer
            gui.sync()
            
            # Sleep to maintain real-time
            time.sleep(0.005)


if __name__ == "__main__":
    main()
