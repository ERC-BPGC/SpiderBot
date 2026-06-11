"""RL configuration for Spiderbot velocity task."""

from mjlab.rl import (
  RslRlModelCfg,
  RslRlOnPolicyRunnerCfg,
  RslRlPpoAlgorithmCfg,
)


def spiderbot_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Create RL runner configuration for Spiderbot velocity task."""
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=False,
      distribution_cfg={
        # Spiderbot's closed-loop linkage is sensitive to rare extreme actions.
        # Use RSL-RL's bounded distribution so sampled actions, rollout storage,
        # action observations, and action-rate rewards all share the same finite
        # support without changing PPO itself.
        "class_name": "BetaDistribution",
        "action_range": (-2.0, 2.0),
      },
    ),
    critic=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=False,
    ),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0,
      use_clipped_value_loss=True,
      clip_param=0.2,
      entropy_coef=0.01,
      num_learning_epochs=5,
      num_mini_batches=4,
      learning_rate=1.0e-3,
      schedule="adaptive",
      gamma=0.99,
      lam=0.95,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    experiment_name="spiderbot_velocity",
    run_name="",
    logger="wandb",
    wandb_project="spiderbot_mjlab",
    wandb_tags=(),
    save_interval=100,
    num_steps_per_env=24,
    max_iterations=10_000,
  )
