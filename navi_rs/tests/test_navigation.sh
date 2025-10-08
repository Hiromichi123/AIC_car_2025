#!/bin/bash

# 发送测试目标点
echo "发送目标点: (5.0, 5.0)"
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'odom'
  },
  pose: {
    position: {x: 1.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

echo "目标已发送！"
echo "监听位置信息："
ros2 topic echo /lidar_data