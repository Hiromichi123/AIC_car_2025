# ROS1 Noetic to ROS2 Humble 迁移完成报告

## 概述
已成功将 `car_bringup` 包从 ROS1 Noetic 迁移到 ROS2 Humble。所有相关的库名称、API 和文件结构都已按照 ROS2 标准进行调整。

## 主要变更

### 1. 构建系统更改
- **ROS1**: 使用 `catkin` 构建系统
- **ROS2**: 现在使用 `ament_cmake` 和 `ament_cmake_python`

### 2. 包配置文件
- `package.xml`: 从格式 2 升级到格式 3
- `CMakeLists.txt`: 完全重写以适配 ament_cmake
- **新增文件**:
  - `setup.py`: Python 包安装配置
  - `setup.cfg`: Python 安装路径配置
  - `resource/car_bringup`: ROS2 资源标记文件

### 3. 依赖项更改

| ROS1 | ROS2 |
|------|------|
| `catkin` | `ament_cmake` |
| `roscpp` | `rclcpp` |
| `rospy` | `rclpy` |
| `tf` | `tf2_ros`, `tf2_geometry_msgs` |
| `dynamic_reconfigure` | 参数 YAML 文件 |

### 4. C++ 代码更改

#### 头文件 (base.h)
- `ros/ros.h` → `rclcpp/rclcpp.hpp`
- `tf/transform_broadcaster.h` → `tf2_ros/transform_broadcaster.h`
- 消息类型现在使用 `::msg::` 命名空间
- 类现在继承自 `rclcpp::Node`

#### 实现文件 (base.cpp)
- 构造函数改为继承 Node
- `nh.param<>()` → `declare_parameter()` 和 `get_parameter()`
- `nh.subscribe()` → `create_subscription<>()`
- `nh.advertise<>()` → `create_publisher<>()`
- `ros::Time::now()` → `this->now()`
- TF 广播更新为使用 TF2

#### 主节点 (base_node.cpp)
- `ros::init()` → `rclcpp::init()`
- `ros::spin()` → `rclcpp::spin()`
- 添加 `rclcpp::shutdown()`

### 5. Python 代码更改

#### 所有 Python 脚本 (odo.py, pubv.py, newt.py)
- `import rospy` → `import rclpy` 和 `from rclpy.node import Node`
- 从基于函数的结构改为基于类的节点结构
- `rospy.Publisher()` → `self.create_publisher()`
- `rospy.Subscriber()` → `self.create_subscription()`
- `print()` → `self.get_logger().info()`
- TF 广播更新为使用 TransformBroadcaster 和 TransformStamped

### 6. 启动文件更改
- **格式**: XML (`.launch`) → Python (`.launch.py`)
- 创建了新的 Python 格式启动文件 `gmapping.launch.py`
- 原始 XML 文件保留作为参考

### 7. 动态参数配置
- ROS1 的 `dynamic_reconfigure` 不再支持
- 创建了 `param/car_params.yaml` 替代 `.cfg` 文件
- 包含所有 PID、校准和巡航参数

## 文件结构

```
car_bringup/
├── car_bringup/          # Python 包目录 (新增)
│   ├── __init__.py
│   ├── odo.py
│   ├── pubv.py
│   └── newt.py
├── include/bringup/
│   └── base.h            # 已更新为 ROS2
├── launch/
│   ├── gmapping.launch   # 原始 ROS1 启动文件
│   └── gmapping.launch.py # 新的 ROS2 启动文件
├── param/
│   ├── car_params.yaml   # 新增：替代 dynamic_reconfigure
│   ├── imu_calib.yaml
│   ├── laser.yaml
│   └── robot_localization.yaml
├── resource/             # 新增：ROS2 资源标记
│   └── car_bringup
├── scripts/              # 原始 Python 脚本（保留作为参考）
│   ├── odo.py
│   ├── pubv.py
│   └── newt.py
├── src/
│   ├── base.cpp          # 已更新为 ROS2
│   └── base_node.cpp     # 已更新为 ROS2
├── CMakeLists.txt        # 重写为 ament_cmake
├── package.xml           # 更新为格式 3
├── setup.py              # 新增：Python 包设置
├── setup.cfg             # 新增：Python 安装配置
├── README.md             # 新增：详细迁移指南
└── MIGRATION_SUMMARY.md  # 新增：变更摘要
```

## 构建和运行

### 前置条件
```bash
# 安装 ROS2 Humble
# 参考: https://docs.ros.org/en/humble/Installation.html

# 安装依赖项
sudo apt update
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install python3-serial  # 用于串口通信
```

### 构建指令
```bash
# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 创建工作空间（如果尚未创建）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 将此包复制到工作空间
# cp -r /path/to/car_bringup .

# 使用 colcon 构建
cd ~/ros2_ws
colcon build --packages-select car_bringup

# 加载工作空间
source install/setup.bash
```

### 运行指令
```bash
# 启动所有节点
ros2 launch car_bringup gmapping.launch.py

# 运行单个节点
ros2 run car_bringup base_node
ros2 run car_bringup pubv
ros2 run car_bringup newt
ros2 run car_bringup odo
```

### 参数设置
```bash
# 从文件加载参数
ros2 param load /odometry_publisher /path/to/car_params.yaml

# 设置单个参数
ros2 param set /odometry_publisher linear_scale_x 1.2

# 获取参数值
ros2 param get /odometry_publisher linear_scale_x
```

## 已知问题和注意事项

1. **外部依赖包**: 启动文件中引用了以下外部包，需要单独迁移：
   - `lslidar_driver`
   - `mowen`
   - `wit_ros_imu`
   - `nav_demo`
   - `robot_localization`
   - `slam_gmapping` (或使用 ROS2 的 `slam_toolbox`)

2. **串口设备**: 脚本假设 `/dev/carserial` 存在。请确保：
   - 创建适当的 udev 规则
   - 确保串口访问权限正确

3. **Dynamic Reconfigure**: `cfg/` 目录中的 `.cfg` 文件保留作为参考，但在 ROS2 中不再使用。参数现在在 `param/car_params.yaml` 中定义。

## API 变更对照表

### C++ API
| ROS1 | ROS2 |
|------|------|
| `ros::init()` | `rclcpp::init()` |
| `ros::NodeHandle nh` | `class Node : public rclcpp::Node` |
| `nh.advertise<T>()` | `create_publisher<T>()` |
| `nh.subscribe()` | `create_subscription<>()` |
| `nh.param<T>()` | `declare_parameter()` + `get_parameter()` |
| `ros::Time::now()` | `this->now()` |
| `ros::spin()` | `rclcpp::spin(node)` |

### Python API
| ROS1 | ROS2 |
|------|------|
| `import rospy` | `import rclpy` + `from rclpy.node import Node` |
| `rospy.init_node()` | `rclpy.init()` + `Node.__init__()` |
| `rospy.Publisher()` | `self.create_publisher()` |
| `rospy.Subscriber()` | `self.create_subscription()` |
| `rospy.spin()` | `rclpy.spin(node)` |
| `print()` | `self.get_logger().info()` |

## 下一步

1. 在 ROS2 Humble 环境中构建和测试包
2. 迁移外部依赖包
3. 使用实际硬件进行测试
4. 根据需要更新其他配置文件

## 相关文档

- 详细的迁移指南: `README.md`
- 完整的变更摘要: `MIGRATION_SUMMARY.md`
- [ROS2 官方迁移指南](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Migration-Guide.html)
- [ROS2 教程](https://docs.ros.org/en/humble/Tutorials.html)

---

迁移已完成！所有核心功能已从 ROS1 Noetic 成功迁移到 ROS2 Humble。
