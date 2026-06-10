"""Spider3d velocity environment configurations."""

from typing import Literal

from mjlab.asset_zoo.robots import (
  SPIDER3D_ACTION_SCALE,
  get_spider3d_robot_cfg,
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
  "last_link_joint_leg_1",
  "last_link_joint_leg_2",
  "last_link_joint_leg_3",
  "last_link_joint_leg_4",
  "last_link_joint_leg_5",
  "last_link_joint_leg_6",
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


def _wire_spider3d_sensors(cfg: ManagerBasedRlEnvCfg) -> None:
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


def spider3d_rough_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """Create Spider3d rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  # Restrict pose reward to actuated joints only (12 total).
  cfg.rewards["pose"].params["asset_cfg"].joint_names = ACTUATED_JOINT_NAMES

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500

  cfg.scene.entities = {"robot": get_spider3d_robot_cfg()}
  _wire_spider3d_sensors(cfg)

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

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = SPIDER3D_ACTION_SCALE

  cfg.viewer.body_name = "base_link"
  cfg.viewer.distance = 1.5
  cfg.viewer.elevation = -10.0

  cfg.observations["actor"].terms["joint_pos"].params = {
    "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  }
  # cfg.observations["actor"].terms["joint_vel"].params = {
  #   "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  # }
  cfg.observations["critic"].terms["joint_pos"].params = {
    "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  }
  cfg.observations["critic"].terms["joint_vel"].params = {
    "asset_cfg": SceneEntityCfg("robot", joint_names=ACTUATED_JOINT_NAMES),
  }

  cfg.events["foot_friction"].params["asset_cfg"].geom_names = FOOT_NAMES
  cfg.events["base_com"].params["asset_cfg"].body_names = ("base_link",)

  # Only calf and parallel_top joints exist in the XML, not hip/thigh
  calf_regex = r"calf_motor_link_joint_leg_\d+"
  parallel_top_regex = r"parallel_link_top_joint_leg_\d+"
  last_link_regex = r"last_link_joint_leg_\d+"
  cfg.rewards["pose"].params["std_standing"] = {
    calf_regex: 0.025,
    parallel_top_regex: 0.025,
    last_link_regex: 0.025,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    calf_regex: 0.5,
    parallel_top_regex: 0.5,
    last_link_regex: 0.5,
  }
  cfg.rewards["pose"].params["std_running"] = {
    calf_regex: 0.5,
    parallel_top_regex: 0.5,
    last_link_regex: 0.5,
  }

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("base_link",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("base_link",)

  cfg.rewards["upright"].params["terrain_sensor_names"] = ("terrain_scan",)
  for reward_name in ["foot_clearance", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = SITE_NAMES

  cfg.rewards["body_ang_vel"].weight = 0.0
  cfg.rewards["angular_momentum"].weight = 0.0
  cfg.rewards["air_time"].weight = 0.0

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


def spider3d_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Spider3d flat terrain velocity configuration."""
  cfg = spider3d_rough_env_cfg(play=play)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
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
    twist_cmd.ranges.lin_vel_x = (0.0, 0.0)
    twist_cmd.ranges.ang_vel_z = (0.0, 0.0)

  return cfg
