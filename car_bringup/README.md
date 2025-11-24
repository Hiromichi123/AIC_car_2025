# ROS1 Noetic to ROS2 Humble Migration Guide

This package has been migrated from ROS1 Noetic to ROS2 Humble.

## Migration Summary

### Package Structure Changes

#### Build System
- **ROS1**: Used `catkin` build system
- **ROS2**: Now uses `ament_cmake` and `ament_cmake_python`

#### Package Files
- `package.xml`: Updated from format 2 to format 3
- `CMakeLists.txt`: Completely rewritten for ament_cmake
- **New files**:
  - `setup.py`: Required for Python packages in ROS2
  - `setup.cfg`: Python installation configuration
  - `resource/car_bringup`: Resource marker file for ROS2

### Dependency Changes

| ROS1 | ROS2 |
|------|------|
| `catkin` | `ament_cmake` |
| `roscpp` | `rclcpp` |
| `rospy` | `rclpy` |
| `tf` | `tf2_ros`, `tf2_geometry_msgs` |
| `dynamic_reconfigure` | ROS2 parameters (YAML files) |

### Code Changes

#### C++ Code
- **Headers**: 
  - `ros/ros.h` → `rclcpp/rclcpp.hpp`
  - `tf/transform_broadcaster.h` → `tf2_ros/transform_broadcaster.h`
  - Message includes now use `.hpp` extension
  - Message types use `::msg::` namespace (e.g., `geometry_msgs::msg::Twist`)

- **Node Structure**:
  - ROS1: Used `ros::NodeHandle`
  - ROS2: Nodes inherit from `rclcpp::Node`

- **Publishers/Subscribers**:
  - ROS1: `nh.advertise<>()` / `nh.subscribe()`
  - ROS2: `create_publisher<>()` / `create_subscription<>()`

- **Parameters**:
  - ROS1: `nh.param<>()`
  - ROS2: `declare_parameter()` and `get_parameter()`

- **Time**:
  - ROS1: `ros::Time::now()`
  - ROS2: `this->now()`

- **Quaternion**:
  - ROS1: `tf::createQuaternionMsgFromYaw()`
  - ROS2: `tf2::Quaternion` with `setRPY()` and `tf2::toMsg()`

#### Python Code
- **Imports**:
  - `rospy` → `rclpy`
  - `import tf` → `from tf2_ros import TransformBroadcaster`

- **Node Structure**:
  - ROS1: Function-based with `rospy.init_node()`
  - ROS2: Class-based inheriting from `Node`

- **Publishers/Subscribers**:
  - ROS1: `rospy.Publisher()` / `rospy.Subscriber()`
  - ROS2: `self.create_publisher()` / `self.create_subscription()`

- **Logging**:
  - ROS1: `print()` or `rospy.loginfo()`
  - ROS2: `self.get_logger().info()`

#### Launch Files
- **Format**: XML (`.launch`) → Python (`.launch.py`)
- **Structure**: Completely different approach
  - ROS1: XML-based declarative
  - ROS2: Python-based programmatic

#### Dynamic Reconfigure
- **ROS1**: Used `dynamic_reconfigure` package with `.cfg` files
- **ROS2**: No direct equivalent; replaced with:
  - YAML parameter files (`param/car_params.yaml`)
  - Parameters can be changed at runtime using `ros2 param set`

### File Structure

```
car_bringup/
├── car_bringup/          # Python package directory (NEW)
│   ├── __init__.py
│   ├── odo.py
│   ├── pubv.py
│   └── newt.py
├── cfg/                  # Dynamic reconfigure configs (DEPRECATED in ROS2)
├── include/
│   └── bringup/
│       └── base.h        # Updated for ROS2
├── launch/
│   ├── gmapping.launch   # Original ROS1 launch file
│   └── gmapping.launch.py # New ROS2 launch file
├── param/
│   ├── car_params.yaml   # NEW: Replaces dynamic_reconfigure
│   ├── imu_calib.yaml
│   ├── laser.yaml
│   └── robot_localization.yaml
├── resource/             # NEW: ROS2 resource marker
│   └── car_bringup
├── rviz/
├── scripts/              # Original Python scripts (kept for reference)
│   ├── odo.py
│   ├── pubv.py
│   └── newt.py
├── src/
│   ├── base.cpp          # Updated for ROS2
│   └── base_node.cpp     # Updated for ROS2
├── CMakeLists.txt        # Rewritten for ament_cmake
├── package.xml           # Updated to format 3
├── setup.py              # NEW: Python package setup
├── setup.cfg             # NEW: Python installation config
└── README.md             # This file
```

## Building the Package

### Prerequisites
```bash
# Install ROS2 Humble
# Follow instructions at: https://docs.ros.org/en/humble/Installation.html

# Install dependencies
sudo apt update
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install python3-serial  # For serial communication
```

### Build Instructions
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Create a workspace (if not already created)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone or copy this package
# cp -r /path/to/car_bringup .

# Build with colcon
cd ~/ros2_ws
colcon build --packages-select car_bringup

# Source the workspace
source install/setup.bash
```

## Running the Package

### Launch the main nodes
```bash
# Launch all nodes (requires external dependencies)
ros2 launch car_bringup gmapping.launch.py

# Run individual nodes
ros2 run car_bringup base_node
ros2 run car_bringup pubv
ros2 run car_bringup newt
ros2 run car_bringup odo
```

### Setting Parameters
```bash
# Load parameters from file
ros2 param load /odometry_publisher /path/to/car_params.yaml

# Set individual parameter
ros2 param set /odometry_publisher linear_scale_x 1.2

# Get parameter value
ros2 param get /odometry_publisher linear_scale_x
```

## Known Issues and Notes

1. **External Dependencies**: The launch file includes references to external packages:
   - `lslidar_driver`
   - `mowen`
   - `wit_ros_imu`
   - `nav_demo`
   - `robot_localization`
   - `slam_gmapping` (or `slam_toolbox` for ROS2)
   
   These are commented out in the launch file and need to be migrated separately.

2. **Serial Port**: The scripts assume `/dev/carserial` exists. Make sure to:
   - Create appropriate udev rules
   - Ensure proper permissions for serial port access

3. **Dynamic Reconfigure**: The `.cfg` files in the `cfg/` directory are kept for reference but are not used in ROS2. Parameters are now defined in `param/car_params.yaml`.

4. **TF Broadcasting**: The transform broadcasting has been updated to use TF2, which is the standard in ROS2.

## Migration Checklist for External Packages

If you have other packages that depend on this one, you'll need to:

- [ ] Update `lslidar_driver` to ROS2
- [ ] Update `mowen` robot model to ROS2
- [ ] Update `wit_ros_imu` to ROS2
- [ ] Update `nav_demo` to ROS2 (or use nav2)
- [ ] Install `robot_localization` for ROS2 Humble
- [ ] Replace `gmapping` with `slam_toolbox` or another ROS2 SLAM package

## Additional Resources

- [ROS2 Migration Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Migration-Guide.html)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Differences between ROS1 and ROS2](https://design.ros2.org/)
