# Publishing Camera’s Data

[Reference](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_camera_publishing.html#isaac-sim-app-tutorial-ros2-camera-publishing)

## Setup VSCode Coding Environment

In Isaac Sim, click on Window > Extensions and enable `omni.isaac.vscode` extension.

Then click on Window > VS Code to open the Isaac Sim folder in a VS Code application.

Source the terminal with:

```bash
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-4.0.0"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```

You can also put the above two lines at the end of ~/.bashrc so you would not need to source everytime you open a new terminal.

Create a `myCode` folder under the isaac-sim-4.0.0 folder:

```bash
~/.local/share/ov/pkg/isaac-sim-4.0.0$ mkdir myCode
cd myCode
```

To structure the project structure better, inside `/home/user1/.local/share/ov/pkg/isaac-sim-4.0.0/myCode`, create a subfolder `5`

```bash
user1@plau-desktop:~/.local/share/ov/pkg/isaac-sim-4.0.0/myCode$ mkdir 5
cd 5
```

Create a file `camera_setup.py` in `/myCode/5`. Copy and paste the code below in `camera_setup.py`.

```py
import carb
from isaacsim import SimulationApp
import sys

BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of stages and manual publishing of images
simulation_app = SimulationApp(CONFIG)
import omni
import numpy as np
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage, extensions, nucleus
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

from omni.isaac.core.utils.prims import set_targets
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims

# Enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)


###### Camera helper functions for setting up publishers. ########

# Paste functions from the tutorial here
# def publish_camera_tf(camera: Camera): ...
# def publish_camera_info(camera: Camera, freq): ...
# def publish_pointcloud_from_depth(camera: Camera, freq): ...
# def publish_depth(camera: Camera, freq): ...
# def publish_rgb(camera: Camera, freq): ...

###################################################################

# Create a Camera prim. Note that the Camera class takes the position and orientation in the world axes convention.
camera = Camera(
    prim_path="/World/floating_camera",
    position=np.array([-3.11, -1.87, 1.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
)
camera.initialize()

simulation_app.update()
camera.initialize()

############### Calling Camera publishing functions ###############

# Call the publishers.
# Make sure you pasted in the helper functions above, and uncomment out the following lines before running.

approx_freq = 30
#publish_camera_tf(camera)
#publish_camera_info(camera, approx_freq)
#publish_rgb(camera, approx_freq)
#publish_depth(camera, approx_freq)
#publish_pointcloud_from_depth(camera, approx_freq)

####################################################################

# Initialize physics
simulation_context.initialize_physics()
simulation_context.play()

while simulation_app.is_running():
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()
```

Run the script using:

```bash
$ISAACSIM_PYTHON_EXE camera_setup.py
```

> I got segmentation error in the first run, but in the second run, I didn't get it anymore

## Next Step

[ROS2 Clock](6_doc.md)

## Previous Step

[ROS 2 Cameras](4_doc.md)