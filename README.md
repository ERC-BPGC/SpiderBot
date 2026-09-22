# Spiderbot MJLab

![Spiderbot hexapod prototype](docs/assets/spidy.jpeg)

This repository contains the simulation and learning codebase for the paper
**Design and Development of an Open-Source Energy-Efficient Hexapod Research
Platform**.

- [Project page and interactive demo](https://maker-rat.github.io/mjlab_spiderbot/)
- Paper: arXiv link will be added when available.

The code is built as a Spiderbot-focused fork of
[mjlab](https://github.com/mujocolab/mjlab). It adds custom hexapod robot assets,
velocity-tracking tasks, and sim2sim XMLs for a closed-loop linkage Spiderbot
platform.

The original MJLab README is preserved as [README_mjlab.md](README_mjlab.md).

## What's Included

- `Spiderbot`: the main closed-loop hexapod with equality constraints and tendon
  spring elements.
- `Spider3d`: a simpler 3-DoF spider-style model used for development and
  comparisons.
- Flat and rough velocity-tracking tasks for both robots.
- Spiderbot-specific terrain, action, contact, and constraint tuning.
- A standalone `sim2sim` MuJoCo XML for checking trained policies outside the
  MJLab task stack.
- `sim2real/`: hardware deployment scripts, legacy ONNX policies, and matching
  standalone simulation assets.
- `Mechanical Design/Hexapod Assembly/`: SolidWorks assemblies, parts, and STEP
  exports. Open `Assem1.SLDASM` with the directory structure intact so referenced
  parts remain available.

## Install

MJLab training requires an NVIDIA GPU. The recommended setup uses
[uv](https://docs.astral.sh/uv/):

```bash
git clone --branch spiderbot-upstream-refresh https://github.com/Maker-Rat/mjlab_spiderbot.git
cd mjlab_spiderbot
uv sync --extra cu128 --group dev
```

If `uv` is not already installed:

```bash
curl -LsSf https://astral.sh/uv/install.sh | UV_INSTALL_DIR="$HOME/.local/bin" sh
```

## Tasks

The added task IDs are:

```text
Mjlab-Velocity-Flat-Spiderbot
Mjlab-Velocity-Rough-Spiderbot
Mjlab-Velocity-Flat-Spider3d
Mjlab-Velocity-Rough-Spider3d
```

## Quick Checks

Run Spiderbot with zero actions:

```bash
uv run play Mjlab-Velocity-Flat-Spiderbot --agent zero
```

Run Spiderbot with random actions:

```bash
uv run play Mjlab-Velocity-Flat-Spiderbot --agent random
```

For rough terrain:

```bash
uv run play Mjlab-Velocity-Rough-Spiderbot --agent zero
uv run play Mjlab-Velocity-Rough-Spiderbot --agent random
```

## Training

Train Spiderbot on rough terrain:

```bash
uv run train Mjlab-Velocity-Rough-Spiderbot --env.scene.num-envs 4096
```

For a smaller first run:

```bash
uv run train Mjlab-Velocity-Rough-Spiderbot --env.scene.num-envs 1024
```

Train on flat terrain:

```bash
uv run train Mjlab-Velocity-Flat-Spiderbot --env.scene.num-envs 4096
```

## Sim2sim

The `sim2sim/` directory contains a standalone MuJoCo setup for Spiderbot policy
validation. The main XML is:

```text
sim2sim/xmls/Hexapod_test.xml
```

Run from the repository root with an exported policy:

```bash
SPIDERBOT_POLICY_PATH=/path/to/exported/policy.onnx uv run python sim2sim/test.py
```

Replace the example path with your ONNX export. The default XML is resolved
relative to the script, so it does not depend on your working directory.
`SPIDERBOT_XML_PATH` optionally selects another scene. The default policy path
`sim2sim/policies/spiderbot.onnx` is a placeholder; a current training checkpoint
is not bundled there. The project page provides a ready-to-use browser demo.

The current sim2sim script uses 40 Hz control, action scale 0.25, and observation
history. Keep the policy, observation layout, XML, and action settings matched.
The older policies in `sim2real/policies/` are not interchangeable with this setup.
The 3-DoF scripts similarly require a separate `flat_3dof.onnx` export.
`SPIDERBOT_OUTPUT_DIR` controls the data collection output directory (default:
`logs/locomotion_data`).

## Hardware deployment

`sim2real/hardware_deploy.py` provides keyboard control; `hardware_deploy_fc.py`
accepts fixed velocity commands. These are legacy hardware controllers using
50 Hz control and action scale 0.5, with no-IMU observations of 27 or 39 values.
They have not been validated with policies trained on the refreshed branch.

The folder includes `rough.onnx`, `rough_noimu.onnx`, and `track_good_20.onnx`.
The original controller referenced `t8.onnx`, which is not included: select a
compatible policy explicitly with `SPIDERBOT_POLICY_PATH`. Confirm its input
layout and calibration before running on the robot.

The standalone controller needs NumPy, ONNX Runtime, pyserial, and the
`scservo_sdk` package. The SDK is not bundled here; the older
[hardware repository](https://github.com/Maker-Rat/Open-Source-Hexapod/tree/main/testing/CPG)
contains the servo code. Install the SDK or set `SPIDERBOT_SERVO_SDK_PATH` to the
directory containing `scservo_sdk`.

Example on a controller with uv and the SDK available:

```bash
SPIDERBOT_POLICY_PATH=/path/to/compatible/no_imu_policy.onnx \
SPIDERBOT_SERVO_SDK_PATH=/path/to/directory/containing/scservo_sdk \
SPIDERBOT_SC_PORT=/dev/ttyACM0 SPIDERBOT_ST_PORT=/dev/ttyACM1 \
uv run --no-project --with numpy --with onnxruntime --with pyserial \
  python sim2real/hardware_deploy.py
```

Review the servo IDs, home positions, tick conversions, and limits at the top of
the script for your assembly. Run from an interactive terminal. The `sim2real`
simulation scripts also support `SPIDERBOT_XML_PATH` and `SPIDERBOT_POLICY_PATH`;
their defaults point to that folder's bundled scenes and `rough.onnx`.

## Tests

Run the velocity task smoke tests:

```bash
uv run pytest tests/test_velocity_task.py
```

## Acknowledgements

This project builds on MJLab, MuJoCo, and MuJoCo Warp. See
[README_mjlab.md](README_mjlab.md) for the original MJLab project description,
documentation links, and upstream usage notes.
