"""Hexapod V2 constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

HEXAPOD_XML: Path = (
  MJLAB_SRC_PATH
  / "asset_zoo"
  / "robots"
  / "spiderbot"
  / "xmls"
  / "Hexapod_V2_full_position.xml"
)
assert HEXAPOD_XML.exists()


def get_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_file(str(HEXAPOD_XML))


##
# Actuator config.
##

# Calf motors
CALF_KP = 10.0
CALF_KV = 1.0
CALF_CTRL_RANGE = (-0.75, 0.75)

# Parallel top motors
PARALLEL_TOP_KP = 10.0
PARALLEL_TOP_KV = 2.5
PARALLEL_TOP_CTRL_RANGE = (-0.4, 0.625)

# Reasonable defaults for missing values
DEFAULT_VEL_LIMIT = 15.0
CALF_EFFORT_LIMIT = 1.5
PARALLEL_TOP_EFFORT_LIMIT = 2.0

# CALF_ACTUATOR = ElectricActuator(
#     reflected_inertia=DEFAULT_ARMATURE,
#     velocity_limit=DEFAULT_VEL_LIMIT,
#     effort_limit=DEFAULT_EFFORT_LIMIT,
# )
# PARALLEL_TOP_ACTUATOR = ElectricActuator(
#     reflected_inertia=DEFAULT_ARMATURE,
#     velocity_limit=DEFAULT_VEL_LIMIT,
#     effort_limit=DEFAULT_EFFORT_LIMIT,
# )

# NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
# DAMPING_RATIO = 2.0

STIFFNESS_CALF = CALF_KP
DAMPING_CALF = CALF_KV

STIFFNESS_PARALLEL_TOP = PARALLEL_TOP_KP
DAMPING_PARALLEL_TOP = PARALLEL_TOP_KV

MIN_LAG = 1
MAX_LAG = 15


SPIDERBOT_CALF_ACTUATOR_DELAYED = BuiltinPositionActuatorCfg(
  target_names_expr=(r"calf_motor_link_joint_leg_\d+",),
  stiffness=STIFFNESS_CALF,
  damping=DAMPING_CALF,
  effort_limit=CALF_EFFORT_LIMIT,
  delay_min_lag=MIN_LAG,
  delay_max_lag=MAX_LAG,
)
SPIDERBOT_PARALLEL_TOP_ACTUATOR_DELAYED = BuiltinPositionActuatorCfg(
  target_names_expr=(r"parallel_link_top_joint_leg_\d+",),
  stiffness=STIFFNESS_PARALLEL_TOP,
  damping=DAMPING_PARALLEL_TOP,
  effort_limit=PARALLEL_TOP_EFFORT_LIMIT,
  delay_min_lag=MIN_LAG,
  delay_max_lag=MAX_LAG,
)

##
# Keyframes.
##

INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.125),
  joint_pos={
    r"calf_motor_link_joint_leg_\d+": 0.0,
    r"parallel_link_top_joint_leg_\d+": 0.0,
  },
  joint_vel={r".*": 0.0},
)

##
# Collision config.
##

FOOT_REGEX = r"^foot_\d+$"

FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(FOOT_REGEX,),
  contype=1,
  conaffinity=1,
  condim=6,
  priority=1,
  solref=(0.01, 1.0),
  friction=(1.0, 0.005, 0.0005),
  solimp=(0.9, 0.95, 0.025),
)

# FULL_COLLISION = CollisionCfg(
#     geom_names_expr=(r".*_collision",),
#     condim={FOOT_REGEX: 6, r".*_collision": 1},
#     priority={FOOT_REGEX: 1},
#     friction=(0.95, 0.02, 0.01),  # Match XML: [sliding, torsional, rolling]
#     solimp=(0.9, 0.95, 0.015),
#     contype=1,
#     conaffinity=0,
# )

##
# Final config.
##

SPIDERBOT_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    SPIDERBOT_CALF_ACTUATOR_DELAYED,
    SPIDERBOT_PARALLEL_TOP_ACTUATOR_DELAYED,
  ),
  soft_joint_pos_limit_factor=0.85,
)


def get_spiderbot_robot_cfg() -> EntityCfg:
  """Get a fresh Spiderbot robot configuration instance."""
  return EntityCfg(
    init_state=INIT_STATE,
    collisions=(FEET_ONLY_COLLISION,),
    spec_fn=get_spec,
    articulation=SPIDERBOT_ARTICULATION,
  )


SPIDERBOT_ACTION_SCALE: dict[str, float] = {}
for a in SPIDERBOT_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  names = a.target_names_expr
  for n in names:
    SPIDERBOT_ACTION_SCALE[n] = 0.25

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_spiderbot_robot_cfg())
  viewer.launch(robot.spec.compile())
