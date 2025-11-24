#!/usr/bin/env python3
#coding=utf-8
import rospy
import tf
from nav_msgs.msg import Odometry

def callback(data):
    # 广播odom到base_link的变换
    broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform(
        (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
        (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
        rospy.Time.now(),
        "base_footprint",
        "odom"
    )

def listener():
    rospy.init_node('odom_to_tf')
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
