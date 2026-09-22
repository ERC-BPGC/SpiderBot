from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  spiderbot_flat_env_cfg,
  spiderbot_rough_env_cfg,
)
from .rl_cfg import spiderbot_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-Spiderbot",
  env_cfg=spiderbot_rough_env_cfg(),
  play_env_cfg=spiderbot_rough_env_cfg(play=True),
  rl_cfg=spiderbot_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Spiderbot",
  env_cfg=spiderbot_flat_env_cfg(),
  play_env_cfg=spiderbot_flat_env_cfg(play=True),
  rl_cfg=spiderbot_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
