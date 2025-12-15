# AIC Car 2025 ROS2 System

This repository contains ROS2 packages for the AIC 2025 car robot platform, supporting both simulation (Gazebo) and real robot deployment.

## 结构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                         Navigation Layer                        │
│  ┌────────────┐      ┌─────────────┐     ┌──────────────┐       │
│  │  navi_rs   │────▶│   /goal     │────▶│  bsp_node    │       │
│  │ (Waypoint  │      │(PoseStamped)│     │ (Position    │       │
│  │ Navigation)│      └─────────────┘     │  Controller) │       │
│  └────────────┘                          └──────┬───────┘       │
│        ▲                                        │               │
│        │                                        ▼               │
│  ┌─────┴──────┐                          ┌──────────────┐       │
│  │/lidar_data │                          │  /cmd_vel    │       │
│  │ (LidarPose)│                          │   (Twist)    │       │
│  └─────┬──────┘                          └──────┬───────┘       │
└────────│────────────────────────────────────────│───────────────┘
         │                                        │
┌────────│────────────────────────────────────────│───────────────────┐
│        │           Middleware Layer             │                   │
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

## 使用

### 仿真模式启动
```bash
# 启动gazebo仿真
ros2 launch robot_gazebo robot.launch.py

# 启动工具
ros2 launch ros2_tools tools_gazebo.launch.py

# 启动视觉节点
ros2 run vision_node vision_gazebo

# 启动导航
ros2 run navi_rs navi_rs
```

### 实机模式启动
```bash
# 启动pointlio导航
ros2 launch robot_real slam.launch.py

# 启动工具
ros2 launch ros2_tools tools_real.launch.py

# 启动视觉节点
ros2 run vision_node vision_real
ros2 run yolip yolip

# 启动导航
ros2 run navi_rs navi_rs
```

#### 三键启动所有（实机）

```bash
ros2 launch robot_real all.launch.py

ros2 run yolip yolip

ros2 run navi_rs navi_rs
```

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

## License

TODO: Add license information
