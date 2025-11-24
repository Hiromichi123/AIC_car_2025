# Migration Summary: ROS1 Noetic → ROS2 Humble

## Overview
This document summarizes the migration of the `car_bringup` package from ROS1 Noetic to ROS2 Humble.

## Files Modified

### Core Configuration Files
1. **package.xml**
   - Changed format from 2 to 3
   - Replaced `catkin` with `ament_cmake` and `ament_cmake_python`
   - Updated dependencies: `roscpp`→`rclcpp`, `rospy`→`rclpy`, `tf`→`tf2_ros`
   - Removed `dynamic_reconfigure` dependency

2. **CMakeLists.txt**
   - Complete rewrite for ament_cmake build system
   - Updated find_package calls for ROS2 dependencies
   - Changed installation paths to ROS2 conventions
   - Added ament_target_dependencies instead of target_link_libraries

### C++ Source Files

3. **include/bringup/base.h**
   - Changed: `#include <ros/ros.h>` → `#include <rclcpp/rclcpp.hpp>`
   - Changed: `#include <tf/transform_broadcaster.h>` → `#include <tf2_ros/transform_broadcaster.h>`
   - Added: `#include <geometry_msgs/msg/twist.hpp>` and other message includes
   - Class now inherits from `rclcpp::Node`
   - Removed NodeHandle members
   - Updated member types for ROS2 (SharedPtr publishers/subscribers)

4. **src/base.cpp**
   - Changed constructor to inherit from Node and use member initializer list
   - Replaced `nh.param<>()` with `declare_parameter()` and `get_parameter()`
   - Changed `nh.subscribe()` → `create_subscription<>()`
   - Changed `nh.advertise<>()` → `create_publisher<>()`
   - Updated callback signature to use SharedPtr
   - Changed `ros::Time::now()` → `this->now()`
   - Changed `tf::createQuaternionMsgFromYaw()` → `tf2::Quaternion` with `setRPY()` and `tf2::toMsg()`
   - Updated time conversion: `.toSec()` → `.seconds()`

5. **src/base_node.cpp**
   - Changed: `ros::init()` → `rclcpp::init()`
   - Changed: `ros::NodeHandle` → `std::make_shared<RobotBase>()`
   - Changed: `ros::spin()` → `rclcpp::spin()`
   - Added: `rclcpp::shutdown()`

### Python Source Files

6. **scripts/odo.py** and **car_bringup/odo.py**
   - Changed: `import rospy` → `import rclpy`
   - Changed: `import tf` → `from tf2_ros import TransformBroadcaster`
   - Converted from function-based to class-based node structure
   - Changed: `rospy.init_node()` → `rclpy.init()` and `Node.__init__()`
   - Changed: `rospy.Subscriber()` → `self.create_subscription()`
   - Changed: `rospy.spin()` → `rclpy.spin()`
   - Updated TF broadcasting to use TransformStamped message

7. **scripts/pubv.py** and **car_bringup/pubv.py**
   - Removed deprecated `roslib.load_manifest()`
   - Changed to class-based node structure
   - Changed: `rospy.Publisher()` → `self.create_publisher()`
   - Converted main loop to timer callback
   - Changed: `print()` → `self.get_logger().info()`
   - Added proper cleanup in destructor

8. **scripts/newt.py** and **car_bringup/newt.py**
   - Removed deprecated `roslib.load_manifest()`
   - Changed to class-based node structure
   - Changed: `rospy.Subscriber()` → `self.create_subscription()`
   - Changed: `print()` → `self.get_logger().info()`
   - Serial object now passed to node constructor
   - Added proper cleanup in main()

### Launch Files

9. **launch/gmapping.launch.py**
   - Created new Python-based launch file (ROS2 format)
   - Original XML launch file kept for reference
   - Converted all node declarations to Python format
   - External package dependencies commented out (need separate migration)
   - Parameters defined inline instead of separate parameter files
   - Remappings handled in Node declarations

### Configuration Files

10. **param/car_params.yaml**
    - New file created to replace dynamic_reconfigure
    - Contains all PID, calibration, and patrol parameters
    - Can be loaded using `ros2 param load` command

### New Files Created

11. **setup.py**
    - Required for Python package in ROS2
    - Defines package metadata
    - Specifies console_scripts entry points
    - Lists data files for installation

12. **setup.cfg**
    - Defines installation paths for Python scripts

13. **resource/car_bringup**
    - Empty marker file required by ROS2 package index

14. **car_bringup/__init__.py**
    - Empty file to make car_bringup a Python package

15. **.gitignore**
    - Ignores build/, install/, log/ directories
    - Ignores Python cache files
    - Ignores checkpoint directories

16. **README.md**
    - Comprehensive migration guide
    - Build and run instructions
    - Known issues and notes

17. **MIGRATION_SUMMARY.md**
    - This file

## Deprecated/Removed Files

- **cfg/*.cfg**: Dynamic reconfigure files no longer used in ROS2
  - Functionality replaced by parameter YAML files
  - Files kept for reference but not used in build

## API Changes Summary

### C++ API Changes
| ROS1 | ROS2 |
|------|------|
| `ros::init()` | `rclcpp::init()` |
| `ros::NodeHandle nh` | `class Node : public rclcpp::Node` |
| `nh.advertise<T>()` | `create_publisher<T>()` |
| `nh.subscribe()` | `create_subscription<>()` |
| `nh.param<T>()` | `declare_parameter()` + `get_parameter()` |
| `ros::Time::now()` | `this->now()` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `geometry_msgs::Twist` | `geometry_msgs::msg::Twist` |
| `tf::TransformBroadcaster` | `tf2_ros::TransformBroadcaster` |
| `tf::createQuaternionMsgFromYaw()` | `tf2::Quaternion::setRPY()` + `tf2::toMsg()` |

### Python API Changes
| ROS1 | ROS2 |
|------|------|
| `import rospy` | `import rclpy` + `from rclpy.node import Node` |
| `rospy.init_node()` | `rclpy.init()` + `Node.__init__()` |
| `rospy.Publisher()` | `self.create_publisher()` |
| `rospy.Subscriber()` | `self.create_subscription()` |
| `rospy.spin()` | `rclpy.spin(node)` |
| `rospy.loginfo()` | `self.get_logger().info()` |
| `import tf` | `from tf2_ros import TransformBroadcaster` |

## Build System Changes

### ROS1
```bash
catkin_make
# or
catkin build
```

### ROS2
```bash
colcon build --packages-select car_bringup
```

## Runtime Changes

### ROS1
```bash
roslaunch car_bringup gmapping.launch
rosrun car_bringup base_node
```

### ROS2
```bash
ros2 launch car_bringup gmapping.launch.py
ros2 run car_bringup base_node
```

## Testing Recommendations

1. **Build Test**: Verify package builds successfully with colcon
2. **Node Tests**: Run each node individually and verify functionality
3. **Integration Tests**: Test complete launch file with all dependencies
4. **Parameter Tests**: Verify parameter loading and runtime modification
5. **TF Tests**: Verify transform broadcasting works correctly
6. **Serial Communication**: Test with actual hardware

## Next Steps

1. Migrate external dependencies:
   - lslidar_driver
   - mowen (robot model)
   - wit_ros_imu
   - nav_demo
   
2. Update or replace deprecated packages:
   - gmapping → slam_toolbox or cartographer
   - robot_localization (verify ROS2 compatibility)
   
3. Test with real hardware
4. Update any additional configuration files
5. Create/update unit tests for ROS2

## Notes

- The original ROS1 files are preserved to maintain reference
- Python scripts exist in both `scripts/` (original) and `car_bringup/` (package) directories
- The launch file has many dependencies commented out - uncomment and update as dependencies are migrated
- Serial port permissions may need to be configured on the target system
