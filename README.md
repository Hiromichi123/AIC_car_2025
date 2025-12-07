# AIC Car 2025 ROS2 System

This repository contains ROS2 packages for the AIC 2025 car robot platform, supporting both simulation (Gazebo) and real robot deployment.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Navigation Layer                            │
│  ┌────────────┐      ┌─────────────┐     ┌──────────────┐           │
│  │  navi_rs   │────▶│   /goal     │────▶│  bsp_node    │           │
│  │ (Waypoint  │      │(PoseStamped)│     │ (Position    │           │
│  │ Navigation)│      └─────────────┘     │  Controller) │           │
│  └────────────┘                          └──────┬───────┘           │
│        ▲                                        │                   │
│        │                                        ▼                   │
│  ┌─────┴──────┐                          ┌──────────────┐           │
│  │/lidar_data │                          │  /cmd_vel    │           │
│  │ (LidarPose)│                          │   (Twist)    │           │
│  └─────┬──────┘                          └──────┬───────┘           │
└────────│────────────────────────────────────────│───────────────────┘
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

## Packages

### ros2_tools
Core ROS2 tools and nodes:
- **lidar_data_node**: odometry -> LidarPose的转换节点
- **bsp_node**: 位置控制转换为速度控制的PID中间层
- **hardware_bridge_node**: 串口桥接层，上下位机通信桥接
- **d435_node**: D435深度相机节点
- **camera_node**: 普通单目摄像头节点

## 使用

### 仿真模式启动
```bash
# 启动gazebo仿真
ros2 launch robot_gazebo robot.launch.py

# 启动工具
ros2 launch ros2_tools tools_gazebo.launch.py

# 启动导航
ros2 run navi_rs navi_rs
```

### 实机模式启动
```bash
# 启动pointlio导航
ros2 launch robot_real slam.launch.py

# 启动工具
ros2 launch ros2_tools tools_real.launch.py serial_port:=/dev/ttyUSB0

# 运行测试节点
ros2 run robot_real simple_goal.launch.py

# 启动导航
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
