#!/usr/bin/env python3
#coding=utf-8

import serial
import roslib; roslib.load_manifest('mbot_bringup')
import rospy
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from math import sqrt, atan2, pow
from threading import Thread
from nav_msgs.msg import Odometry
import time

def callback(msg):
    cmd_twist_rotation = msg.angular.z
    cmd_twist_x = msg.linear.x
    cmd_twist_y = msg.linear.y
    th_1 = cmd_twist_rotation * 1000
    x_1 = cmd_twist_x * 1000
    y_1 = cmd_twist_y * 1000
    if int(x_1) < 0:
        x_2 = int(x_1) & 0xffff
        x_2 = str(hex(x_2))
    else:
        x_2 = str(hex(int(x_1)))
    num_x_2 = len(x_2)
    if num_x_2 < 4:
        send_x_2 = (x_2[-1:])
        x_2 = '000' + send_x_2
    if num_x_2 == 4:
        send_x_2 = (x_2[-2:])
        x_2 = '00' + send_x_2
    if num_x_2 == 5:
        send_x_2 = (x_2[-3:])
        x_2 = '0' + send_x_2
    if num_x_2 == 6:
        send_x_2 = (x_2[-4:])
        x_2 = send_x_2
    x_2_1 = (x_2[:2])
    x_2_2 = (x_2[-2:])
    x_2_1 = bytes.fromhex(x_2_1)
    x_2_2 = bytes.fromhex(x_2_2)
    print (x_2_1,x_2_2)
    if int(y_1) < 0:
        y_2 = int(y_1) & 0xffff
        y_2 = str(hex(y_2))
    else:
        y_2 = str(hex(int(y_1)))
    num_y_2 = len(y_2)
    if num_y_2 < 4:
        send_y_2 = (y_2[-1:])
        y_2 = '000' + send_y_2
    if num_y_2 == 4:
        send_y_2 = (y_2[-2:])
        y_2 = '00' + send_y_2
    if num_y_2 == 5:
        send_y_2 = (y_2[-3:])
        y_2 = '0' + send_y_2
    if num_y_2 == 6:
        send_y_2 = (y_2[-4:])
        y_2 = send_y_2
    y_2_1 = (y_2[:2])
    y_2_2 = (y_2[-2:])
    y_2_1 = bytes.fromhex(y_2_1)
    y_2_2 = bytes.fromhex(y_2_2)
    print (y_2_1,y_2_2)
    if int(th_1) < 0:
        th_2 = int(th_1) & 0xffff
        th_2 = str(hex(th_2))
    else:
        th_2 = str(hex(int(th_1)))
    num_th_2 = len(th_2)
    if num_th_2 < 4:
        send_th_2 = (th_2[-1:])
        th_2 = '000' + send_th_2
    if num_th_2 == 4:
        send_th_2 = (th_2[-2:])
        th_2 = '00' + send_th_2
    if num_th_2 == 5:
        send_th_2 = (th_2[-3:])
        th_2 = '0' + send_th_2
    if num_th_2 == 6:
        send_th_2 = (th_2[-4:])
        th_2 = send_th_2
    th_2_1 = (th_2[:2])
    th_2_2 = (th_2[-2:])
    th_2_1 = bytes.fromhex(th_2_1)
    th_2_2 = bytes.fromhex(th_2_2)
    print (th_2_1,th_2_2)
    ser.write(b'\xAA')
    time.sleep(0.0004)
    ser.write(b'\xBB')
    time.sleep(0.0004)
    ser.write(b'\x0A')
    time.sleep(0.0004)
    ser.write(b'\x12')
    time.sleep(0.0004)
    ser.write(b'\x02')
    time.sleep(0.0004)
    ser.write(x_2_2)
    time.sleep(0.0004)
    ser.write(x_2_1)
    time.sleep(0.0004)
    ser.write(y_2_2)
    time.sleep(0.0004)
    ser.write(y_2_1)
    time.sleep(0.0004)
    ser.write(th_2_2)
    time.sleep(0.0004)
    ser.write(th_2_1)
    time.sleep(0.0004)
    ser.write(b'\x00')
    time.sleep(0.0004)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    # rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    print('serial')
    ser = serial.Serial("/dev/carserial",115200,timeout = 5)
    if ser.is_open:
        print('open_success')
    ser.write(b'\x11')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    ser.write(b'\x00')
    time.sleep(0.001)
    print('init_success')
    listener()
