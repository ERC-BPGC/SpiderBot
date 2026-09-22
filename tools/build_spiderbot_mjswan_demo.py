"""Build a static mjswan Spiderbot browser demo.

This is an experimental bridge for the project page. It builds a model-only
MuJoCo WASM viewer first, so we can validate Spiderbot's equality constraints
and tendon spring behavior before adding ONNX policy control.
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MJSWAN_SRC = REPO_ROOT.parent / "mjswan" / "src"
DEFAULT_ONNX_PATH = (
  REPO_ROOT
  / "logs"
  / "rsl_rl"
  / "spiderbot_velocity"
  / "2026-06-11_14-45-38"
  / "2026-06-11_14-45-38.onnx"
)
ACTUATED_JOINT_NAMES = (
  "calf_motor_link_joint_leg_1",
  "parallel_link_top_joint_leg_1",
  "calf_motor_link_joint_leg_2",
  "parallel_link_top_joint_leg_2",
  "calf_motor_link_joint_leg_3",
  "parallel_link_top_joint_leg_3",
  "calf_motor_link_joint_leg_4",
  "parallel_link_top_joint_leg_4",
  "calf_motor_link_joint_leg_5",
  "parallel_link_top_joint_leg_5",
  "calf_motor_link_joint_leg_6",
  "parallel_link_top_joint_leg_6",
)


def _add_mjswan_to_path() -> None:
  mjswan_src = Path(os.environ.get("MJSWAN_SRC", DEFAULT_MJSWAN_SRC)).expanduser()
  if not mjswan_src.exists():
    raise FileNotFoundError(
      f"Could not find mjswan source at {mjswan_src}. "
      "Set MJSWAN_SRC=/path/to/mjswan/src if it is cloned elsewhere."
    )
  sys.path.insert(0, str(mjswan_src))


def _patch_mjswan_nodeenv_path() -> None:
  """Ensure npm subprocesses can find the node binary from nodeenv.

  mjswan invokes the nodeenv-local npm executable by absolute path. On this
  machine npm's shebang uses ``/usr/bin/env node``, so the nodeenv bin directory
  still needs to be on PATH.
  """
  # Optional source checkout resolved at runtime through MJSWAN_SRC.
  ClientBuilder = importlib.import_module("mjswan._build_client").ClientBuilder

  original_install_dependencies = ClientBuilder.install_dependencies
  original_run_build_script = ClientBuilder.run_build_script

  def _with_nodeenv_path(builder: Any) -> str:
    nodeenv_bin = str(
      builder.nodeenv_dir / ("Scripts" if sys.platform == "win32" else "bin")
    )
    old_path = os.environ.get("PATH", "")
    os.environ["PATH"] = f"{nodeenv_bin}{os.pathsep}{old_path}"
    return old_path

  def install_dependencies(self: Any, clean: bool = False) -> None:
    old_path = _with_nodeenv_path(self)
    try:
      original_install_dependencies(self, clean=clean)
    finally:
      os.environ["PATH"] = old_path

  def run_build_script(
    self: Any,
    script_name: str = "build",
    env: dict[str, str] | None = None,
  ) -> None:
    old_path = _with_nodeenv_path(self)
    try:
      original_run_build_script(self, script_name=script_name, env=env)
    finally:
      os.environ["PATH"] = old_path

  ClientBuilder.install_dependencies = install_dependencies
  ClientBuilder.run_build_script = run_build_script


def _patch_mjswan_history_order() -> None:
  """Patch the local mjswan template to flatten history oldest-to-newest.

  mjlab's exported Spiderbot policy was validated through sim2sim with
  term-major chronological history ordering. mjswan 0.6.0 emits per-term
  histories newest-to-oldest, so patch the local cloned template before the
  frontend build. The replacements are idempotent.
  """
  observations_ts = (
    Path(sys.path[0])
    / "mjswan"
    / "template"
    / "src"
    / "core"
    / "observation"
    / "observations.ts"
  )
  if not observations_ts.exists():
    raise FileNotFoundError(f"mjswan observations.ts not found at {observations_ts}")

  text = observations_ts.read_text()
  text = text.replace(
    "for (const buffer of this.history) {\n      output.set(buffer, offset);",
    "for (const buffer of [...this.history].reverse()) {\n      output.set(buffer, offset);",
  )
  text = text.replace(
    "flattened[j * this.steps + i] = this.actionBuffer[i][j];",
    "flattened[j * this.steps + i] = this.actionBuffer[this.steps - 1 - i][j];",
  )
  text = text.replace(
    "flattened[i * this.numActions + j] = this.actionBuffer[i][j];",
    "flattened[i * this.numActions + j] = this.actionBuffer[this.steps - 1 - i][j];",
  )
  observations_ts.write_text(text)


def main() -> None:
  _add_mjswan_to_path()

  mjswan = importlib.import_module("mjswan")
  import onnx

  JointPositionActionCfg = importlib.import_module(
    "mjswan.envs.mdp.actions"
  ).JointPositionActionCfg

  import mjlab.tasks  # noqa: F401 - populate the mjlab task registry.
  from mjlab.tasks.registry import load_env_cfg

  _patch_mjswan_nodeenv_path()
  _patch_mjswan_history_order()

  output_dir = REPO_ROOT / "docs" / "project-page" / "interactive"
  onnx_path = Path(os.environ.get("SPIDERBOT_ONNX", DEFAULT_ONNX_PATH)).expanduser()
  if not onnx_path.exists():
    raise FileNotFoundError(
      f"Spiderbot ONNX policy not found at {onnx_path}. "
      "Set SPIDERBOT_ONNX=/path/to/policy.onnx to build policy control."
    )
  task_id = "Mjlab-Velocity-Flat-Spiderbot"
  env_cfg = load_env_cfg(task_id, play=True)
  env_cfg.observations["actor"].terms["joint_pos"].params["pos_steps"] = [2, 1, 0]
  policy_joint_names = [f"robot/{name}" for name in ACTUATED_JOINT_NAMES]
  default_joint_pos = [0.0 for _ in ACTUATED_JOINT_NAMES]

  builder = mjswan.Builder(
    base_path="/mjlab_spiderbot/interactive/",
    debug=True,
  )
  project = builder.add_project(name="Spiderbot Interactive Demo")
  scene = project.add_mjlab_scene(task_id, play=True)
  scene.set_viewer_config(
    mjswan.ViewerConfig(
      lookat=(0.0, 0.0, 0.05),
      distance=1.4,
      elevation=-12.0,
      azimuth=35.0,
      origin_type=mjswan.ViewerConfig.OriginType.ASSET_BODY,
      body_name="base_link",
      enable_reflections=False,
      enable_shadows=True,
      width=960,
      height=620,
    )
  )
  scene.add_policy(
    name="Flat Policy",
    policy=onnx.load(onnx_path),
    observations={"policy": env_cfg.observations["actor"]},
    commands={
      "twist": mjswan.velocity_command(
        lin_vel_x=(-0.5, 0.5),
        lin_vel_y=(-0.5, 0.5),
        ang_vel_z=(-0.7, 0.7),
        default_lin_vel_x=0.0,
        default_lin_vel_y=0.0,
        default_ang_vel_z=0.0,
      )
    },
    actions={
      "joint_pos": JointPositionActionCfg(
        entity_name="robot",
        actuator_names=policy_joint_names,
        scale={name: 0.25 for name in policy_joint_names},
        use_default_offset=True,
      )
    },
    policy_joint_names=policy_joint_names,
    default_joint_pos=default_joint_pos,
    default=True,
  )

  app = builder.build(output_dir)
  del app
  print(f"Built Spiderbot mjswan demo at: {output_dir}")


if __name__ == "__main__":
  main()
