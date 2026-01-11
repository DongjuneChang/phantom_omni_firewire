# Phantom Omni ROS2 Driver (FireWire)

ROS2 Humble driver for **Sensable Phantom Omni** haptic device via FireWire (IEEE 1394).

## Requirements

- **SDK**: OpenHaptics AE 3.0 + JUJU FireWire Driver
- **Hardware**: Sensable Phantom Omni (FireWire version) + Thunderbolt adapter
- **ROS2**: Humble (Ubuntu 22.04)

> **Why OpenHaptics AE 3.0?**  
> Modern haptic devices (Geomagic Touch) use USB, so OpenHaptics 3.4+ dropped FireWire support.
> Many labs still have the original Phantom Omni with FireWire (IEEE 1394) interface.
> This driver uses **OpenHaptics AE 3.0** with **JUJU driver** to support these legacy devices on modern Linux.

## Docker Usage (Recommended)

Docker setup instructions coming soon.

## Native Build

```bash
# Prerequisites: OpenHaptics AE 3.0 + JUJU driver installed
# See: https://github.com/jhu-cisst-external/phantom-omni-1394-drivers

source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
# Clone this repository
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
# Driver only
ros2 launch omni_common omni_state.launch.py

# With RViz visualization
ros2 launch omni_common omni_state.launch.py rviz:=true
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/phantom/joint_states` | sensor_msgs/JointState | 6 joint positions |
| `/phantom/pose` | geometry_msgs/PoseStamped | End-effector pose |
| `/phantom/state` | omni_msgs/OmniState | Full state (pos, orient, vel) |
| `/phantom/button` | omni_msgs/OmniButtonEvent | Button events |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/phantom/calibrate` | std_srvs/Trigger | Set current position as zero |

## Configuration

Joint calibration in `omni_common/config/phantom_omni.yaml`:

```yaml
omni_haptic_node:
  ros__parameters:
    joint_scales: [-1.0, 1.0, 1.0, -1.0, -1.0, -1.0]
    joint_offsets: [0.0, 0.0, 0.0, 3.14159, -2.35619, -3.14159]
```

## Packages

| Package | Description |
|---------|-------------|
| `omni_common` | Driver node, launch files, config |
| `omni_description` | URDF model and meshes |
| `omni_msgs` | Custom message types |

## Acknowledgments

This project is based on the work of several contributors:

- **Francisco Suárez Ruiz** ([@fsuarez6](https://github.com/fsuarez6))  
  Original ROS1 driver: [fsuarez6/phantom_omni](https://github.com/fsuarez6/phantom_omni)

- **JHU CISST External**  
  OpenHaptics AE 3.0 JUJU FireWire drivers: [jhu-cisst-external/phantom-omni-1394-drivers](https://github.com/jhu-cisst-external/phantom-omni-1394-drivers)

- **Sensable Technologies** (now 3D Systems)  
  Original Phantom Omni hardware and OpenHaptics SDK

## License

BSD-3-Clause (same as original ROS1 package)

## Author

- **Dongjune Chang** (dongjune.chang@gmail.com)  
  ROS2 Humble port, Docker integration, modernization
