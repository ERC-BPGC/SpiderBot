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
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "spider3d" / "xmls" / "3dof.xml"
)
assert HEXAPOD_XML.exists()


def get_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_file(str(HEXAPOD_XML))


##
# Actuator config.
##

# Calf motors
CALF_KP = 12.5
CALF_KV = 0.075
CALF_CTRL_RANGE = (-0.75, 0.75)

# Parallel top motors
PARALLEL_TOP_KP = 20.0
PARALLEL_TOP_KV = 0.07
PARALLEL_TOP_CTRL_RANGE = (-0.4, 0.625)

LAST_LINK_KP = 20.0
LAST_LINK_KV = 0.07
LAST_LINK_CTRL_RANGE = (-4.0, 0.75)

# Reasonable defaults for missing values
DEFAULT_VEL_LIMIT = 10.0

PARALLEL_TOP_EFFORT_LIMIT = 3.0
CALF_EFFORT_LIMIT = 1.5
LAST_LINK_EFFORT_LIMIT = 3.0

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

STIFFNESS_LAST_LINK = LAST_LINK_KP
DAMPING_LAST_LINK = LAST_LINK_KV

MIN_LAG = 1
MAX_LAG = 10


SPIDER3D_CALF_ACTUATOR_DELAYED = BuiltinPositionActuatorCfg(
  target_names_expr=(r"calf_motor_link_joint_leg_\d+",),
  stiffness=STIFFNESS_CALF,
  damping=DAMPING_CALF,
  effort_limit=CALF_EFFORT_LIMIT,
  delay_min_lag=MIN_LAG,
  delay_max_lag=MAX_LAG,
)
SPIDER3D_PARALLEL_TOP_ACTUATOR_DELAYED = BuiltinPositionActuatorCfg(
  target_names_expr=(r"parallel_link_top_joint_leg_\d+",),
  stiffness=STIFFNESS_PARALLEL_TOP,
  damping=DAMPING_PARALLEL_TOP,
  effort_limit=PARALLEL_TOP_EFFORT_LIMIT,
  delay_min_lag=MIN_LAG,
  delay_max_lag=MAX_LAG,
)

SPIDER3D_LAST_LINK_ACTUATOR_DELAYED = BuiltinPositionActuatorCfg(
  target_names_expr=(r"last_link_joint_leg_\d+",),
  stiffness=STIFFNESS_LAST_LINK,
  damping=DAMPING_LAST_LINK,
  effort_limit=LAST_LINK_EFFORT_LIMIT,
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
    r"last_link_joint_leg_\d+": 0.0,
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
  conaffinity=0,
  condim=3,
  priority=1,
  solref=(0.01, 1.0),
  friction=(0.8,),
  solimp=(0.9, 0.95, 0.023),  # was (0.9, 0.95, 0.023)
)

##
# Final config.
##

SPIDER3D_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    SPIDER3D_CALF_ACTUATOR_DELAYED,
    SPIDER3D_PARALLEL_TOP_ACTUATOR_DELAYED,
    SPIDER3D_LAST_LINK_ACTUATOR_DELAYED,
  ),
  soft_joint_pos_limit_factor=0.85,
)


def get_spider3d_robot_cfg() -> EntityCfg:
  """Get a fresh Spider3D robot configuration instance."""
  return EntityCfg(
    init_state=INIT_STATE,
    collisions=(FEET_ONLY_COLLISION,),
    spec_fn=get_spec,
    articulation=SPIDER3D_ARTICULATION,
  )


SPIDER3D_ACTION_SCALE: dict[str, float] = {}
for a in SPIDER3D_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  names = a.target_names_expr
  for n in names:
    SPIDER3D_ACTION_SCALE[n] = 0.5

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_spider3d_robot_cfg())
  viewer.launch(robot.spec.compile())
