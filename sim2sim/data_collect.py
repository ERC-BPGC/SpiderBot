"""
Sim2Sim inference script for Hexapod policy.
Loads an ONNX model and runs it in pure MuJoCo.

Modified to track:
1. Foot workspace for leg 1 when in contact
2. Mean and std of all last link positions
"""

import mujoco
from mujoco import viewer
import numpy as np
import time
import onnxruntime as ort
from collections import deque
import pickle
import os

# ============================================================================
# Configuration
# ============================================================================

XML_PATH = "/media/marmot/606de469-2f76-4155-82bc-e2e657636ad7/Ritwik/mjlab_alt/sim2sim/xmls/flat_3dof.xml"
ONNX_MODEL_PATH = "/media/marmot/606de469-2f76-4155-82bc-e2e657636ad7/Ritwik/mjlab_alt/sim2sim/policies/flat_3dof.onnx"

CONTROL_FREQ = 50  # Hz (policy runs at 50 Hz)
ACTION_SCALE = 0.5  # Scale applied to policy outputs

# Output file paths
OUTPUT_DIR = "/home/marmot/claude/locomotion_data"
FOOT_WORKSPACE_FILE = "foot_workspace_leg1.pkl"
LAST_LINK_STATS_FILE = "last_link_stats.pkl"

# Actuated joint names (18 joints - 3 per leg)
ACTUATED_JOINT_NAMES = [
    "calf_motor_link_joint_leg_1", "parallel_link_top_joint_leg_1", "last_link_joint_leg_1",
    "calf_motor_link_joint_leg_2", "parallel_link_top_joint_leg_2", "last_link_joint_leg_2",
    "calf_motor_link_joint_leg_3", "parallel_link_top_joint_leg_3", "last_link_joint_leg_3",
    "calf_motor_link_joint_leg_4", "parallel_link_top_joint_leg_4", "last_link_joint_leg_4",
    "calf_motor_link_joint_leg_5", "parallel_link_top_joint_leg_5", "last_link_joint_leg_5",
    "calf_motor_link_joint_leg_6", "parallel_link_top_joint_leg_6", "last_link_joint_leg_6",
]

# Site names for tracking
FOOT_SITE_LEG1 = "foot_site_2"
LAST_LINK_SITES = [f"site1_leg_{i}" for i in range(1, 7)]

# Command limits (for keyboard control)
MAX_LIN_VEL_X = 0.5  # m/s
MAX_LIN_VEL_Y = 0.5  # m/s
MAX_ANG_VEL_Z = 0.7  # rad/s

# Contact detection threshold
CONTACT_FORCE_THRESHOLD = 0.1  # Minimum normal force to consider foot in contact

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
    upvector = get_sensor_data(model, data, "upvector")
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
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        
        found = False
        for act_id in range(model.nu):
            if model.actuator_trntype[act_id] == mujoco.mjtTrn.mjTRN_JOINT:
                trnid = model.actuator_trnid[act_id, 0]
                if trnid == joint_id:
                    actuator_indices.append(act_id)
                    found = True
                    break
        
        if not found:
            raise ValueError(f"No actuator found for joint {joint_name}")
    
    return np.array(actuator_indices)


def get_site_position_in_base_frame(model, data, site_name, base_body_name="base_link"):
    """
    Get site position in base frame coordinates.
    Returns position as [x, y, z] in base frame.
    """
    # Get site ID and position in world frame
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    site_pos_world = data.site_xpos[site_id].copy()
    
    # Get base body ID and position/orientation in world frame
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base_body_name)
    base_pos_world = data.xpos[base_id].copy()
    base_quat = data.xquat[base_id].copy()  # quaternion [w, x, y, z]
    
    # Convert position from world to base frame
    # 1. Translate: position relative to base
    pos_relative = site_pos_world - base_pos_world
    
    # 2. Rotate: use conjugate of base quaternion to rotate into base frame
    # Convert quaternion to rotation matrix
    base_mat = np.zeros(9)
    mujoco.mju_quat2Mat(base_mat, base_quat)
    base_mat = base_mat.reshape(3, 3)
    
    # Rotate into base frame (inverse rotation = transpose of rotation matrix)
    pos_in_base = base_mat.T @ pos_relative
    
    return pos_in_base


def apply_30deg_rotation(points):
    """
    Apply 30-degree rotation around Z-axis.
    Args:
        points: numpy array of shape (N, 3) or (3,)
    Returns:
        rotated points with same shape
    """
    angle = np.deg2rad(0)
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    
    if points.ndim == 1:
        return rotation_matrix @ points
    else:
        return (rotation_matrix @ points.T).T


def is_foot_in_contact(model, data, foot_geom_name):
    """
    Check if a foot is in contact with the ground.
    Returns True if contact force exceeds threshold.
    """
    # Get foot geom ID
    foot_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, foot_geom_name)
    
    # Check all contacts
    for i in range(data.ncon):
        contact = data.contact[i]
        
        # Check if this contact involves our foot
        if contact.geom1 == foot_geom_id or contact.geom2 == foot_geom_id:
            # Get contact force
            # Contact forces are in data.efc_force, but we need to find the right index
            # Simpler approach: check if contact is active and has reasonable penetration
            if contact.dist < 0:  # Penetration indicates contact
                return True
    
    return False


class KeyboardCommandInterface:
    """Handle keyboard input for velocity commands."""
    
    def __init__(self, max_lin_x, max_lin_y, max_ang_z):
        self.max_lin_x = max_lin_x
        self.max_lin_y = max_lin_y
        self.max_ang_z = max_ang_z
        
        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel_z = 0.0
        
        self.increment = 0.1
        self.ang_increment = 0.1
        
    def update(self, key_presses):
        """Update commands based on key presses."""
        if key_presses.get('w', False):
            self.lin_vel_x = min(self.lin_vel_x + self.increment, self.max_lin_x)
        if key_presses.get('s', False):
            self.lin_vel_x = max(self.lin_vel_x - self.increment, -self.max_lin_x)
        
        if key_presses.get('a', False):
            self.lin_vel_y = min(self.lin_vel_y + self.increment, self.max_lin_y)
        if key_presses.get('d', False):
            self.lin_vel_y = max(self.lin_vel_y - self.increment, -self.max_lin_y)
        
        if key_presses.get('o', False):
            self.ang_vel_z = min(self.ang_vel_z + self.ang_increment, self.max_ang_z)
        if key_presses.get('p', False):
            self.ang_vel_z = max(self.ang_vel_z - self.ang_increment, -self.max_ang_z)
        
        if key_presses.get('space', False):
            self.lin_vel_x = 0.0
            self.lin_vel_y = 0.0
            self.ang_vel_z = 0.0
        
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


class DataTracker:
    """Track locomotion data for analysis."""
    
    def __init__(self, output_dir):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Foot workspace data for leg 1
        self.foot_workspace_leg1 = []  # List of (x, y, z) positions when in contact
        
        # Last link position data for all legs
        self.last_link_positions = []  # List of arrays of shape (6, 3)
        
    def record_foot_contact(self, model, data, in_contact):
        """Record foot position when in contact."""
        if in_contact:
            # Get foot position in base frame
            foot_pos_base = get_site_position_in_base_frame(model, data, FOOT_SITE_LEG1)
            
            # Apply 30-degree rotation
            foot_pos_rotated = apply_30deg_rotation(foot_pos_base)
            
            self.foot_workspace_leg1.append(foot_pos_rotated.copy())
    
    def record_last_link_positions(self, model, data):
        """Record positions of all last links."""
        positions = []
        for site_name in LAST_LINK_SITES:
            pos = get_site_position_in_base_frame(model, data, site_name)
            positions.append(pos)
        
        self.last_link_positions.append(np.array(positions))
    
    def save_data(self):
        """Save all tracked data to files."""
        # Save foot workspace
        foot_workspace_path = os.path.join(self.output_dir, FOOT_WORKSPACE_FILE)
        with open(foot_workspace_path, 'wb') as f:
            pickle.dump({
                'positions': np.array(self.foot_workspace_leg1),
                'description': 'Foot positions in base frame (rotated 30 deg) when in contact',
                'coordinate_frame': 'base_link with 30-degree Z-axis rotation applied'
            }, f)
        print(f"✅ Saved foot workspace data to {foot_workspace_path}")
        print(f"   Total contact points recorded: {len(self.foot_workspace_leg1)}")
        
        # Compute and save last link statistics
        if len(self.last_link_positions) > 0:
            all_positions = np.array(self.last_link_positions)  # Shape: (N_samples, 6_legs, 3_xyz)
            
            # Compute mean and std for each leg
            mean_positions = np.mean(all_positions, axis=0)  # Shape: (6, 3)
            std_positions = np.std(all_positions, axis=0)    # Shape: (6, 3)
            
            # Overall statistics across all legs
            overall_mean = np.mean(all_positions, axis=(0, 1))  # Shape: (3,)
            overall_std = np.std(all_positions, axis=(0, 1))    # Shape: (3,)
            
            last_link_stats_path = os.path.join(self.output_dir, LAST_LINK_STATS_FILE)
            with open(last_link_stats_path, 'wb') as f:
                pickle.dump({
                    'mean_per_leg': mean_positions,
                    'std_per_leg': std_positions,
                    'overall_mean': overall_mean,
                    'overall_std': overall_std,
                    'all_positions': all_positions,
                    'n_samples': len(self.last_link_positions),
                    'description': 'Last link positions in base frame',
                    'coordinate_frame': 'base_link'
                }, f)
            print(f"✅ Saved last link statistics to {last_link_stats_path}")
            print(f"   Number of samples: {len(self.last_link_positions)}")
            print(f"   Overall mean position: [{overall_mean[0]:.4f}, {overall_mean[1]:.4f}, {overall_mean[2]:.4f}]")
            print(f"   Overall std deviation: [{overall_std[0]:.4f}, {overall_std[1]:.4f}, {overall_std[2]:.4f}]")
    
    def print_summary(self):
        """Print a summary of collected data."""
        print("\n" + "="*60)
        print("DATA COLLECTION SUMMARY")
        print("="*60)
        print(f"Foot workspace (Leg 1) contact points: {len(self.foot_workspace_leg1)}")
        print(f"Last link position samples: {len(self.last_link_positions)}")
        
        if len(self.foot_workspace_leg1) > 0:
            workspace = np.array(self.foot_workspace_leg1)
            print("\nFoot Workspace Statistics (Leg 1, rotated 30°):")
            print(f"  X range: [{workspace[:, 0].min():.4f}, {workspace[:, 0].max():.4f}] m")
            print(f"  Y range: [{workspace[:, 1].min():.4f}, {workspace[:, 1].max():.4f}] m")
            print(f"  Z range: [{workspace[:, 2].min():.4f}, {workspace[:, 2].max():.4f}] m")
        
        if len(self.last_link_positions) > 0:
            all_pos = np.array(self.last_link_positions)
            overall_mean = np.mean(all_pos, axis=(0, 1))
            overall_std = np.std(all_pos, axis=(0, 1))
            print("\nLast Link Statistics (all legs):")
            print(f"  Mean position: [{overall_mean[0]:.4f}, {overall_mean[1]:.4f}, {overall_mean[2]:.4f}] m")
            print(f"  Std deviation: [{overall_std[0]:.4f}, {overall_std[1]:.4f}, {overall_std[2]:.4f}] m")
        
        print("="*60 + "\n")


# ============================================================================
# Main Inference Loop
# ============================================================================

def main():
    # Load MuJoCo model
    print(f"Loading MuJoCo model from {XML_PATH}...")
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    
    # Load ONNX model
    print(f"Loading ONNX model from {ONNX_MODEL_PATH}...")
    ort_session = ort.InferenceSession(ONNX_MODEL_PATH)
    
    # Get joint and actuator indices
    qpos_indices, qvel_indices = get_joint_indices(model, ACTUATED_JOINT_NAMES)
    actuator_indices = get_actuator_indices(model, ACTUATED_JOINT_NAMES)
    
    print(f"Number of actuated joints: {len(ACTUATED_JOINT_NAMES)}")
    
    # Get default joint positions
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    default_joint_pos = data.qpos[qpos_indices].copy()
    
    # Initialize command interface and data tracker
    cmd_interface = KeyboardCommandInterface(MAX_LIN_VEL_X, MAX_LIN_VEL_Y, MAX_ANG_VEL_Z)
    data_tracker = DataTracker(OUTPUT_DIR)
    
    # Action history
    last_action = np.zeros(len(ACTUATED_JOINT_NAMES), dtype=np.float32)
    
    # Control timing
    dt_control = 1.0 / CONTROL_FREQ
    dt_sim = model.opt.timestep
    n_sim_steps = int(dt_control / dt_sim)
    
    print(f"\nControl frequency: {CONTROL_FREQ} Hz")
    print(f"Simulation timestep: {dt_sim*1000:.1f} ms")
    print(f"Simulation steps per control step: {n_sim_steps}")
    print(f"\nData will be saved to: {OUTPUT_DIR}")
    print("\n" + "="*60)
    print("Starting data collection...")
    print("="*60 + "\n")
    
    # Launch viewer
    with viewer.launch_passive(model, data) as gui:
        # MANUAL COMMAND SETTING
        TEST_FORWARD_AFTER_STEPS = 100
        MAX_STEPS = 1000  # Run for a limited time to collect data
        
        step_count = 0
        last_control_time = time.time()
        
        try:
            while gui.is_running() and step_count < MAX_STEPS:
                current_time = time.time()
                
                # Control at specified frequency
                if current_time - last_control_time >= dt_control:
                    
                    # MANUAL COMMAND
                    if step_count == TEST_FORWARD_AFTER_STEPS:
                        print(f"\n{'='*60}")
                        print(f"ACTIVATING FORWARD COMMAND at step {step_count}")
                        print(f"{'='*60}\n")
                        cmd_interface.lin_vel_y = 0.35
                        cmd_interface.ang_vel_z = 0.0
                    
                    # ================================================================
                    # 1. Construct Observation
                    # ================================================================
                    
                    base_ang_vel = get_sensor_data(model, data, "imu_ang_vel")
                    projected_gravity = get_projected_gravity(model, data)
                    joint_pos = data.qpos[qpos_indices] - default_joint_pos
                    joint_vel = data.qvel[qvel_indices] * 0.05
                    actions_obs = last_action.copy()
                    command = cmd_interface.get_command()
                    
                    observation = np.concatenate([
                        base_ang_vel,
                        projected_gravity,
                        joint_pos,
                        joint_vel,
                        actions_obs,
                        command,
                    ]).astype(np.float32)
                    
                    # ================================================================
                    # 2. Run Policy Inference
                    # ================================================================
                    
                    obs_batch = observation.reshape(1, -1)
                    ort_inputs = {ort_session.get_inputs()[0].name: obs_batch}
                    ort_outputs = ort_session.run(None, ort_inputs)
                    action = ort_outputs[0][0]
                    
                    action = action * ACTION_SCALE
                    action = np.clip(action, -1.0, 1.0)
                    last_action = action.copy()
                    
                    # ================================================================
                    # 3. Apply Actions
                    # ================================================================
                    
                    target_positions = default_joint_pos + action
                    data.ctrl[actuator_indices] = target_positions
                    
                    # ================================================================
                    # 4. Data Collection
                    # ================================================================
                    
                    # Check if foot 1 is in contact and record position
                    foot_in_contact = is_foot_in_contact(model, data, "foot_1")
                    data_tracker.record_foot_contact(model, data, foot_in_contact)
                    
                    # Record last link positions for all legs
                    data_tracker.record_last_link_positions(model, data)
                    
                    # ================================================================
                    # 5. Print Debug Info
                    # ================================================================
                    
                    if step_count % 50 == 0:
                        print(f"Step {step_count}/{MAX_STEPS}")
                        print(f"  Command: [{command[0]:+.2f}, {command[1]:+.2f}, {command[2]:+.2f}]")
                        print(f"  Foot 1 in contact: {foot_in_contact}")
                        print(f"  Data points collected: {len(data_tracker.foot_workspace_leg1)}")
                        print()
                    
                    step_count += 1
                    last_control_time = current_time
                
                # Step simulation
                mujoco.mj_step(model, data)
                gui.sync()
                time.sleep(0.0001)
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user!")
        
        finally:
            # Save data at the end
            print("\n" + "="*60)
            print("Saving collected data...")
            print("="*60 + "\n")
            data_tracker.save_data()
            data_tracker.print_summary()


if __name__ == "__main__":
    main()