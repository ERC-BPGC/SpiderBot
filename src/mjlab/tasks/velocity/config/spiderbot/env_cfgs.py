"""Spiderbot velocity environment configurations."""

from typing import Literal

from mjlab.asset_zoo.robots import (
  SPIDERBOT_ACTION_SCALE,
  get_spiderbot_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import (
  ContactMatch,
  ContactSensorCfg,
  ObjRef,
  RayCastSensorCfg,
  RingPatternCfg,
  TerrainHeightSensorCfg,
)
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise

TerrainType = Literal["rough", "obstacles"]

ACTUATED_JOINT_NAMES = (
  "calf_motor_link_joint_leg_1",
  "calf_motor_link_joint_leg_2",
  "calf_motor_link_joint_leg_3",
  "calf_motor_link_joint_leg_4",
  "calf_motor_link_joint_leg_5",
  "calf_motor_link_joint_leg_6",
  "parallel_link_top_joint_leg_1",
  "parallel_link_top_joint_leg_2",
  "parallel_link_top_joint_leg_3",
  "parallel_link_top_joint_leg_4",
  "parallel_link_top_joint_leg_5",
  "parallel_link_top_joint_leg_6",
)
FOOT_NAMES = ("foot_1", "foot_2", "foot_3", "foot_4", "foot_5", "foot_6")
SITE_NAMES = (
  "foot_site_1",
  "foot_site_2",
  "foot_site_3",
  "foot_site_4",
  "foot_site_5",
  "foot_site_6",
)


def _wire_spiderbot_sensors(cfg: ManagerBasedRlEnvCfg) -> None:
  """Set upstream generic velocity sensors to Spiderbot frames."""
  for sensor in cfg.scene.sensors or ():
    if sensor.name == "terrain_scan":
      assert isinstance(sensor, RayCastSensorCfg)
      assert isinstance(sensor.frame, ObjRef)
      sensor.frame.name = "base_link"
    elif sensor.name == "foot_height_scan":
      assert isinstance(sensor, TerrainHeightSensorCfg)
      sensor.frame = tuple(
        ObjRef(type="site", name=site_name, entity="robot") for site_name in SITE_NAMES
      )
      sensor.pattern = RingPatternCfg.single_ring(radius=0.04, num_samples=4)


def _apply_spiderbot_velocity_tuning(cfg: ManagerBasedRlEnvCfg) -> None:
  """Preserve old Spiderbot-only tuning from the former shared velocity config.

  This intentionally lives in the Spiderbot config rather than changing mjlab's
  upstream velocity defaults.
  """
  actor_terms = cfg.observations["actor"].terms
  actor_terms.pop("base_lin_vel", None)
  actor_terms.pop("height_scan", None)

  actor_terms["joint_pos"].params = {
    "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  }
  actor_terms["joint_pos"].noise = Unoise(n_min=-0.025, n_max=0.025)
  actor_terms["joint_pos"].clip = (-5.0, 5.0)
  actor_terms["joint_pos"].delay_min_lag = 1
  actor_terms["joint_pos"].delay_max_lag = 6
  actor_terms["joint_pos"].history_length = 3
  actor_terms["joint_vel"].params = {
    "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  }
  actor_terms["joint_vel"].scale = 0.05
  actor_terms["joint_vel"].clip = (-5.0, 5.0)
  actor_terms["joint_vel"].history_length = 3
  actor_terms["actions"].history_length = 3
  cfg.observations["actor"].terms = {
    name: actor_terms[name]
    for name in (
      "base_ang_vel",
      "projected_gravity",
      "joint_pos",
      "joint_vel",
      "actions",
      "command",
    )
  }

  critic_terms = cfg.observations["critic"].terms
  critic_terms["base_lin_vel"].clip = (-10.0, 10.0)
  critic_terms["base_lin_vel"].history_length = 3
  critic_terms["base_ang_vel"].clip = (-10.0, 10.0)
  critic_terms["base_ang_vel"].history_length = 3
  critic_terms["projected_gravity"].clip = (-1.5, 1.5)
  critic_terms["projected_gravity"].history_length = 3
  critic_terms["joint_pos"].params = {
    "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  }
  critic_terms["joint_pos"].clip = (-5.0, 5.0)
  critic_terms["joint_pos"].history_length = 3
  critic_terms["joint_vel"].params = {
    "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  }
  critic_terms["joint_vel"].scale = 0.05
  critic_terms["joint_vel"].clip = (-5.0, 5.0)
  critic_terms["joint_vel"].history_length = 3
  critic_terms["actions"].history_length = 3
  critic_terms["height_scan"].clip = (-1.0, 1.0)
  critic_terms["height_scan"].history_length = 3
  critic_terms["foot_height"].clip = (-2.0, 2.0)
  critic_terms["foot_height"].history_length = 3
  critic_terms["foot_air_time"].clip = (0.0, 25.0)
  critic_terms["foot_air_time"].history_length = 3
  critic_terms["foot_contact"].history_length = 3
  critic_terms["foot_contact_forces"].clip = (-1000.0, 1000.0)
  critic_terms["foot_contact_forces"].history_length = 3

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = SPIDERBOT_ACTION_SCALE
  joint_pos_action.clip = {
    r"calf_motor_link_joint_leg_\d+": (-0.65, 0.65),
    r"parallel_link_top_joint_leg_\d+": (-0.35, 0.55),
  }

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.ranges.lin_vel_x = (-0.5, 0.5)
  twist_cmd.ranges.lin_vel_y = (-0.5, 0.5)
  twist_cmd.ranges.ang_vel_z = (-0.7, 0.7)

  cfg.events["reset_base"].params["velocity_range"] = {
    "x": (-0.3, 0.3),
    "y": (-0.2, 0.2),
    "yaw": (-0.5, 0.5),
  }
  cfg.events["reset_robot_joints"].params["position_range"] = (-0.05, 0.05)
  cfg.events["reset_robot_joints"].params["velocity_range"] = (-0.01, 0.01)
  cfg.events["randomize_rigid_body_mass_base"] = EventTermCfg(
    mode="startup",
    func=envs_mdp.dr.body_mass,
    params={
      "asset_cfg": SceneEntityCfg("robot", body_names="^base_link$"),
      "ranges": (-1.0, 1.0),
      "operation": "add",
    },
  )
  cfg.events["randomize_rigid_body_mass_others"] = EventTermCfg(
    mode="startup",
    func=envs_mdp.dr.body_mass,
    params={
      "asset_cfg": SceneEntityCfg("robot", body_names="^(?!base_link$).*"),
      "ranges": (0.8, 1.2),
      "operation": "scale",
    },
  )
  cfg.events["randomize_motor_offset"] = EventTermCfg(
    mode="startup",
    func=envs_mdp.dr.joint_default_pos,
    params={
      "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      "ranges": (-0.035, 0.035),
      "operation": "add",
    },
  )
  cfg.events["randomize_actuator_gains"] = EventTermCfg(
    mode="reset",
    func=envs_mdp.dr.pd_gains,
    params={
      "asset_cfg": SceneEntityCfg("robot"),
      "kp_range": (0.8, 1.2),
      "kd_range": (0.8, 1.2),
      "operation": "scale",
    },
  )
  cfg.events["randomize_joint_armature"] = EventTermCfg(
    mode="startup",
    func=envs_mdp.dr.joint_armature,
    params={
      "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
      "ranges": (0.75, 2.5),
      "operation": "scale",
    },
  )

  cfg.rewards["track_linear_velocity"].weight = 7.5
  cfg.rewards["track_linear_velocity"].params["std"] = 0.1**0.5
  cfg.rewards["track_angular_velocity"].params["std"] = 0.50**0.5
  # This reward is computed on raw policy actions, not the scaled/clipped joint
  # targets. Keep it moderate so a brief exploration spike cannot dominate the
  # return and destabilize PPO.
  cfg.rewards["action_rate_l2"].weight = -0.1
  cfg.rewards["air_time"].weight = 1.0
  cfg.rewards["air_time"].params["threshold_min"] = 0.25
  cfg.rewards["air_time"].params["threshold_max"] = 0.75
  cfg.rewards["air_time"].params["command_threshold"] = 0.05
  cfg.rewards["foot_clearance"].weight = -2.5
  cfg.rewards["foot_clearance"].params["target_height"] = 0.065
  cfg.rewards["foot_swing_height"].params["target_height"] = 0.065

  cfg.curriculum["command_vel"].params["velocity_stages"] = [
    {
      "step": 0,
      "lin_vel_x": (-0.1, 0.1),
      "lin_vel_y": (-0.1, 0.1),
      "ang_vel_z": (-0.4, 0.4),
    },
    {
      "step": 1500 * 24,
      "lin_vel_x": (-0.18, 0.18),
      "lin_vel_y": (-0.18, 0.18),
      "ang_vel_z": (-0.5, 0.5),
    },
    {
      "step": 2500 * 24,
      "lin_vel_x": (-0.25, 0.25),
      "lin_vel_y": (-0.25, 0.25),
      "ang_vel_z": (-0.55, 0.55),
    },
    {
      "step": 3500 * 24,
      "lin_vel_x": (-0.4, 0.4),
      "lin_vel_y": (-0.4, 0.4),
      "ang_vel_z": (-0.7, 0.7),
    },
    {"step": 5500 * 24, "lin_vel_x": (-0.5, 0.5), "lin_vel_y": (-0.5, 0.5)},
    {"step": 7500 * 24, "lin_vel_x": (-0.6, 0.6), "lin_vel_y": (-0.5, 0.5)},
  ]

  # The closed-loop leg linkage is represented by equality constraints. Keep the
  # constraint softness itself sim2sim-compatible, but give MuJoCo more solver
  # budget so the linkage stays tighter under abrupt random actions.
  cfg.sim.mujoco.iterations = 50
  cfg.sim.mujoco.ls_iterations = 50

  cfg.decimation = 5


def _apply_spiderbot_terrain_tuning(cfg: ManagerBasedRlEnvCfg) -> None:
  """Reduce rough terrain difficulty for Spiderbot without changing presets."""
  if cfg.scene.terrain is None or cfg.scene.terrain.terrain_generator is None:
    return
  sub_terrains = cfg.scene.terrain.terrain_generator.sub_terrains
  sub_terrains["pyramid_stairs"].step_height_range = (0.0, 0.05)
  sub_terrains["pyramid_stairs"].step_width = 0.5
  sub_terrains["pyramid_stairs_inv"].step_height_range = (0.0, 0.05)
  sub_terrains["pyramid_stairs_inv"].step_width = 0.5
  sub_terrains["hf_pyramid_slope"].slope_range = (0.0, 0.35)
  sub_terrains["hf_pyramid_slope_inv"].slope_range = (0.0, 0.35)
  sub_terrains["random_rough"].noise_range = (0.0, 0.05)
  sub_terrains["random_rough"].noise_step = 0.005
  sub_terrains["wave_terrain"].amplitude_range = (0.0, 0.065)


def spiderbot_rough_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """Create Spiderbot rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  # Restrict pose reward to actuated joints only (12 total).
  cfg.rewards["pose"].params["asset_cfg"].joint_names = ACTUATED_JOINT_NAMES

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500

  cfg.scene.entities = {"robot": get_spiderbot_robot_cfg()}
  _wire_spiderbot_sensors(cfg)
  _apply_spiderbot_velocity_tuning(cfg)
  _apply_spiderbot_terrain_tuning(cfg)

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="geom", pattern=FOOT_NAMES, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  cfg.scene.sensors = (cfg.scene.sensors or ()) + (
    feet_ground_cfg,
  )

  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True

  cfg.viewer.body_name = "base_link"
  cfg.viewer.distance = 1.5
  cfg.viewer.elevation = -10.0

  cfg.events["foot_friction"].params["asset_cfg"].geom_names = FOOT_NAMES
  cfg.events["base_com"].params["asset_cfg"].body_names = ("base_link",)

  # Only calf and parallel_top joints exist in the XML, not hip/thigh
  calf_regex = r"calf_motor_link_joint_leg_\d+"
  parallel_top_regex = r"parallel_link_top_joint_leg_\d+"
  cfg.rewards["pose"].params["std_standing"] = {
    calf_regex: 0.05,
    parallel_top_regex: 0.05,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    calf_regex: 0.5,
    parallel_top_regex: 0.5,
  }
  cfg.rewards["pose"].params["std_running"] = {
    calf_regex: 0.5,
    parallel_top_regex: 0.5,
  }

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("base_link",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("base_link",)

  cfg.rewards["upright"].params["terrain_sensor_names"] = ("terrain_scan",)
  for reward_name in ["foot_clearance", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = SITE_NAMES

  cfg.rewards["body_ang_vel"].weight = 0.0
  cfg.rewards["angular_momentum"].weight = 0.0
  cfg.rewards["air_time"].weight = 1.0

  # Disabled nonfoot_ground_cfg and related termination due to missing geoms.
  # cfg.terminations["illegal_contact"] = TerminationTermCfg(
  #   func=mdp.illegal_contact,
  #   params={"sensor_name": nonfoot_ground_cfg.name},
  # )

  # Apply play mode overrides.
  if play:
    # Effectively infinite episode length.
    cfg.episode_length_s = int(1e9)

    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum = {}
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=envs_mdp.randomize_terrain,
      mode="reset",
      params={},
    )

    if cfg.scene.terrain is not None:
      if cfg.scene.terrain.terrain_generator is not None:
        cfg.scene.terrain.terrain_generator.curriculum = False
        cfg.scene.terrain.terrain_generator.num_cols = 5
        cfg.scene.terrain.terrain_generator.num_rows = 5
        cfg.scene.terrain.terrain_generator.border_width = 10.0

  return cfg


def spiderbot_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Spiderbot flat terrain velocity configuration."""
  cfg = spiderbot_rough_env_cfg(play=play)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 5
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  # Switch to flat terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # Remove raycast sensor and height scan (no terrain to scan).
  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"
  )
  cfg.observations["actor"].terms.pop("height_scan", None)
  cfg.observations["critic"].terms.pop("height_scan", None)
  cfg.rewards["upright"].params.pop("terrain_sensor_names", None)

  # Disable terrain curriculum (not present in play mode since rough clears all).
  cfg.curriculum.pop("terrain_levels", None)

  if play:
    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    twist_cmd.ranges.lin_vel_x = (-0.5, 0.5)
    twist_cmd.ranges.ang_vel_z = (-0.7, 0.7)

  return cfg
