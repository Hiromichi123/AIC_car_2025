cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/) # 安装 launch 文件
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/) # 安装 world 文件
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/) # 安装 URDF 文件

ament_package()
