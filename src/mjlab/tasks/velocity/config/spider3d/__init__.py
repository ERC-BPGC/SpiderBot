from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  spider3d_rough_env_cfg,
  spider3d_flat_env_cfg,
)
from .rl_cfg import spider3d_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-Spider3d",
  env_cfg=spider3d_rough_env_cfg(),
  play_env_cfg=spider3d_rough_env_cfg(play=True),
  rl_cfg=spider3d_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Spider3d",
  env_cfg=spider3d_flat_env_cfg(),
  play_env_cfg=spider3d_flat_env_cfg(play=True),
  rl_cfg=spider3d_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
