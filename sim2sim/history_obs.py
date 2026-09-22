"""Observation history helpers for sim2sim policy playback."""

from collections import deque

import numpy as np


class ObservationHistory:
  """Chronological oldest-to-newest history, matching MJLab flattening."""

  def __init__(self, history_length: int) -> None:
    if history_length < 1:
      raise ValueError(f"history_length must be >= 1, got {history_length}")
    self.history_length = history_length
    self._buffers: dict[str, deque[np.ndarray]] = {}

  def reset(self) -> None:
    self._buffers.clear()

  def stack(self, name: str, value: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=np.float32).reshape(-1)
    if name not in self._buffers:
      self._buffers[name] = deque(
        [value.copy() for _ in range(self.history_length)],
        maxlen=self.history_length,
      )
    else:
      self._buffers[name].append(value.copy())
    return np.concatenate(list(self._buffers[name]), axis=0)


def build_actor_observation(
  history: ObservationHistory,
  base_ang_vel: np.ndarray,
  projected_gravity: np.ndarray,
  joint_pos: np.ndarray,
  joint_vel: np.ndarray,
  actions_obs: np.ndarray,
  command: np.ndarray,
) -> np.ndarray:
  """Build Spiderbot actor observation in the trained term order."""

  return np.concatenate(
    [
      history.stack("base_ang_vel", base_ang_vel),
      history.stack("projected_gravity", projected_gravity),
      history.stack("joint_pos", joint_pos),
      history.stack("joint_vel", joint_vel),
      history.stack("actions", actions_obs),
      np.asarray(command, dtype=np.float32).reshape(-1),
    ],
    axis=0,
  ).astype(np.float32)


def process_policy_action(
  raw_action: np.ndarray,
  action_scale: float,
  raw_clip: float | None = 2.0,
  processed_clip: tuple[np.ndarray, np.ndarray] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
  """Return raw action for observation history and processed joint delta."""

  raw_action = np.asarray(raw_action, dtype=np.float32).reshape(-1)
  if raw_clip is not None:
    raw_action = np.clip(raw_action, -raw_clip, raw_clip)

  processed_action = raw_action * action_scale
  if processed_clip is not None:
    lower, upper = processed_clip
    processed_action = np.clip(processed_action, lower, upper)

  return raw_action.astype(np.float32), processed_action.astype(np.float32)
