ros2 topic pub /goal geometry_msgs/msg/PoseStamped "{ \
header: {frame_id: 'map'}, \
pose: { \
position: {x: 2.0, y: 1.0, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} \
} \
}" -r 20