#!/bin/bash

trap "kill 0" SIGINT SIGTERM

# 启动Livox驱动 + Point-LIO + TF广播，不启动 RViz
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 5

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link livox_frame &
ros2 launch point_lio point_lio.launch.py rviz:=True &
