cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(odom_plugin SHARED plugin/odom_plugin.cpp)
ament_target_dependencies(odom_plugin rclcpp nav_msgs gazebo)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/) # 安装 launch 文件
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/) # 安装 world 文件
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/) # 安装 URDF 文件
install(TARGETS odom_plugin LIBRARY DESTINATION lib) # 安装插件

ament_package()
