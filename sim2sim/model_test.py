import os
import time
from pathlib import Path

import mujoco
from mujoco import viewer

# Path to your MuJoCo XML model (make sure assets are relative or absolute correctly)
xml_path = os.environ.get(
  "SPIDERBOT_XML_PATH",
  str(Path(__file__).resolve().parent / "xmls" / "flat_terrain.xml"),
)

# Load model and data
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

joint_names = []
for i in range(model.njnt):
  joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
  if joint_name:  # Check if a name exists (some joints might not be explicitly named)
    joint_names.append(joint_name)

lowers, uppers = model.jnt_range[1:].T
print("Joint limits (lower, upper):", lowers.shape, uppers.shape)

print("List of joints:", joint_names)

# Launch viewer (GUI window)
with viewer.launch_passive(model, data) as gui:
  print("✅ Viewer launched. Press ESC to quit.")

  # Step and render simulation
  while gui.is_running():
    mujoco.mj_step(model, data)
    gui.sync()
    time.sleep(0.0001)
