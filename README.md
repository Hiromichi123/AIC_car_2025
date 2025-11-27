# AIC Car 2025 ROS2 System

This repository contains ROS2 packages for the AIC 2025 car robot platform, supporting both simulation (Gazebo) and real robot deployment.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Navigation Layer                             │
│  ┌────────────┐     ┌─────────────┐     ┌──────────────┐           │
│  │  navi_rs   │────▶│   /goal     │────▶│  bsp_node    │           │
│  │ (Waypoint  │     │(PoseStamped)│     │ (Position    │           │
│  │  Navigation)│     └─────────────┘     │  Controller) │           │
│  └────────────┘                          └──────┬───────┘           │
│        ▲                                        │                    │
│        │                                        ▼                    │
│  ┌─────┴──────┐                          ┌──────────────┐           │
│  │/lidar_data │                          │  /cmd_vel    │           │
│  │ (LidarPose)│                          │   (Twist)    │           │
│  └─────┬──────┘                          └──────┬───────┘           │
└────────│────────────────────────────────────────│───────────────────┘
         │                                        │
┌────────│────────────────────────────────────────│───────────────────┐
│        │           Middleware Layer             │                    │
│  ┌─────┴──────────┐                   ┌─────────┴────────────────┐  │
│  │lidar_data_node │                   │                          │  │
│  │                │                   │  SIMULATION    REAL      │  │
│  │ Converts odom  │                   │      │          │        │  │
│  │ to LidarPose   │                   │      ▼          ▼        │  │
│  └────────────────┘                   │  ┌──────┐  ┌──────────┐  │  │
│                                       │  │Gazebo│  │hardware_ │  │  │
│                                       │  │Plugin│  │bridge_   │  │  │
│                                       │  └──────┘  │node      │  │  │
│                                       │            └──────────┘  │  │
│                                       └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## Packages

### ros2_tools
Core ROS2 tools and nodes:
- **lidar_data_node**: Converts odometry data to unified LidarPose format
- **bsp_node**: Position controller using PID for goal tracking
- **hardware_bridge_node**: Serial communication bridge for real robot motor control
- **d435_node**: Intel RealSense D435 camera interface
- **ground_camera_node**: Ground-facing camera interface

### navi_rs
Rust-based navigation system with:
- Waypoint navigation
- YOLO/OCR service integration
- Goal publishing to bsp_node

### robot_gazebo
Gazebo simulation environment:
- Mecanum wheel robot model
- World files
- Gazebo odometry plugin

### vision_node
Computer vision services:
- YOLO object detection
- OCR text recognition
- Camera image processing

## Usage

### Simulation Mode
```bash
# Start Gazebo simulation
ros2 launch robot_gazebo robot.launch.py

# Start ROS2 tools (in another terminal)
ros2 launch ros2_tools tools.launch.py

# Run navigation (in another terminal)
ros2 run navi_rs navi_rs
```

### Real Robot Mode
```bash
# Start PointLIO for odometry (if using lidar)
# ... (lidar SLAM launch)

# Start ROS2 tools for real robot
ros2 launch ros2_tools real_robot.launch.py serial_port:=/dev/ttyUSB0

# Run navigation (in another terminal)
ros2 run navi_rs navi_rs
```

## Configuration

### lidar_data_node Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_simulation` | `true` | Use simulation or real robot mode |
| `simulation_odom_topic` | `/absolute_pose` | Gazebo odometry topic |
| `real_robot_odom_topic` | `/aft_mapped_to_init` | PointLIO odometry topic |

### hardware_bridge_node Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port for motor controller |
| `baud_rate` | `115200` | Serial communication baud rate |
| `enable_serial` | `true` | Enable serial communication |
| `max_linear_speed` | `1.0` | Maximum linear speed (m/s) |
| `max_angular_speed` | `2.0` | Maximum angular speed (rad/s) |
| `wheel_radius` | `0.05` | Wheel radius in meters |
| `robot_length` | `0.3` | Robot length (front-back) in meters |
| `robot_width` | `0.25` | Robot width (left-right) in meters |

## Serial Protocol

The `hardware_bridge_node` communicates with the motor controller using a simple serial protocol:

### Packet Format
```
[Header 1][Header 2][Command][Payload...][Checksum]
   0xAA      0x55     1 byte   N bytes    1 byte
```

### Commands
- `0x01`: Set wheel velocities (8 bytes payload: 4x 16-bit signed integers)
- `0x02`: Emergency stop (0 bytes payload)
- `0x03`: Query status (0 bytes payload)

### Wheel Order
1. Front Left
2. Front Right
3. Rear Left
4. Rear Right

## License

TODO: Add license information
