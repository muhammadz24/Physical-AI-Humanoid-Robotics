---
sidebar_position: 3
title: Setup Guide & Robot Import
---

# Setup Guide & Robot Import

## Installation Steps (Ubuntu 22.04 + RTX 4070 Ti)

### Step 1: Verify GPU and Drivers

```bash
# Check GPU
nvidia-smi

# Expected: RTX 4070 Ti, Driver 535+, CUDA 12.x

# If driver missing:
sudo ubuntu-drivers install nvidia:535
sudo reboot
```

### Step 2: Download Omniverse Launcher

```bash
# Download from NVIDIA
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

### Step 3: Install Isaac Sim via Launcher

1. Open Omniverse Launcher
2. Navigate to **Exchange** tab
3. Search "Isaac Sim" → Install version **2023.1.1** (~20GB download)
4. Installation path: `~/.local/share/ov/pkg/isaac_sim-2023.1.1/`

### Step 4: Launch and Verify

```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

**First Launch**:
- Compiles shaders (takes 5-10 minutes on first run)
- Opens Isaac Sim window with sample scenes

**Verify GPU Acceleration**:
- Window → Viewport → Stats
- Should show: **GPU: RTX 4070 Ti**, **PhysX: GPU**, **60+ FPS**

## Importing URDF Robots

### Method 1: URDF Importer Extension

**Enable Extension**:
1. Window → Extensions
2. Search "URDF Importer"
3. Enable extension

**Import URDF**:
```python
from omni.isaac.urdf import _urdf

# Import Unitree Go2
urdf_path = "/path/to/go2_description/urdf/go2.urdf"
imported_robot = _urdf.acquire_urdf_interface()

result = imported_robot.parse_urdf(
    urdf_path,
    "/World/Go2",  # USD path
    {
        "create_physics_scene": True,
        "import_inertia_tensor": True,
        "distance_scale": 1.0,  # URDF in meters
        "density": 1000.0,  # kg/m³ for missing masses
    }
)

print(f"Import successful: {result}")
```

**Result**: Go2 quadruped appears in scene with articulations configured.

### Method 2: Manual USD Creation (Advanced)

**Convert URDF → USD**:
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh \
  omni/isaac/urdf/scripts/urdf_converter.py \
  --urdf-path /path/to/robot.urdf \
  --usd-path /path/to/output/robot.usd \
  --make-instanceable
```

**Load in Isaac Sim**:
```python
from pxr import Usd, UsdGeom

stage = omni.usd.get_context().get_stage()
robot_prim = stage.DefinePrim("/World/Robot", "Xform")

# Reference converted USD
robot_prim.GetReferences().AddReference("file:///path/to/robot.usd")
```

## Configuring Sensors

### RealSense D435i Camera

```python
from omni.isaac.sensor import Camera

realsense = Camera(
    prim_path="/World/Go2/base_link/realsense",
    frequency=30,
    resolution=(1920, 1080),
    position=[0.3, 0, 0.1],  # Front of robot
    orientation=[1, 0, 0, 0]  # Forward-facing
)

# Enable depth
realsense.add_depth_data_to_frame()

# Get data
rgb = realsense.get_rgba()[:, :, :3]
depth = realsense.get_depth()  # meters
```

### 3D LIDAR (Velodyne VLP-16)

```python
from omni.isaac.range_sensor import LidarRtx

lidar = LidarRtx(
    prim_path="/World/Go2/base_link/lidar",
    config="Velodyne_VLP16",
)

# Customize parameters
lidar.set_horizontal_fov(360.0)  # degrees
lidar.set_vertical_fov(30.0)
lidar.set_horizontal_resolution(0.4)  # degrees/sample
lidar.set_rotation_frequency(10.0)  # Hz

# Get point cloud
points = lidar.get_point_cloud_data()  # Nx3 numpy array
```

**RTX Ray Tracing**: LIDAR uses RT cores for ~60× faster ray casting vs CPU.

## ROS 2 Bridge Setup

### Enable ROS 2 Extension

1. Window → Extensions → Search "ROS 2 Bridge"
2. Enable extension
3. ROS 2 → Settings:
   - Domain ID: 0 (default)
   - RMW Implementation: cyclonedds

### Publish Topics from Isaac Sim

**Python Script**:

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import omni.graph.core as og

# Create ROS 2 publisher graph
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("PublishImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("CameraHelper.outputs:execOut", "PublishImage.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("CameraHelper.inputs:cameraPrim", "/World/Go2/base_link/realsense"),
            ("PublishImage.inputs:topicName", "/realsense/image_raw"),
            ("PublishImage.inputs:frameId", "camera_link"),
        ],
    },
)
```

**Verify in ROS 2**:
```bash
# Terminal 1: Launch Isaac Sim (run script above)

# Terminal 2: Check topics
ros2 topic list
# Should show: /realsense/image_raw

# Terminal 3: View image
ros2 run rqt_image_view rqt_image_view /realsense/image_raw
```

### Subscribe to ROS 2 Commands

```python
# Create ROS 2 subscriber for cmd_vel
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph_Cmd", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
            ("SubTwist.outputs:linearVelocity", "ArticulationController.inputs:velocityCommand"),
        ],
        keys.SET_VALUES: [
            ("SubTwist.inputs:topicName", "/cmd_vel"),
            ("ArticulationController.inputs:robotPath", "/World/Go2"),
        ],
    },
)
```

**Control from ROS 2**:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Use arrow keys → Go2 moves in Isaac Sim!
```

---

**Next**: [Synthetic Data Generation](./synthetic-data) for massive-scale AI training datasets.
