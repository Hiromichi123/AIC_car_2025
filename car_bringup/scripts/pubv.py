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
from geometry_msgs.msg import Twist
rospy.init_node('vel_raw_pub')
velPublisher = rospy.Publisher("/vel_raw", Twist, queue_size=100)
# 打开串口
if __name__ == '__main__':
    print('serial')
    ser = serial.Serial("/dev/carserial",115200,timeout = 5)
    if ser.is_open:
        print('open_success')
    while not rospy.is_shutdown():
        raw_data = ser.read(12)
        if len(raw_data) >0: 
            for i in range(len(raw_data)-1):
                if raw_data[i]== 0xAA and raw_data[i+1]== 0xBB :
                    if i!=0:
                        break
                    twist = Twist()
                    # 检查字节流中是否包含协议数据头
                    #print(raw_data[0:12])
                    a1=raw_data[i+2]
                    a2=raw_data[i+3]
                    xl=raw_data[i+4]
                    xh=raw_data[i+5]
                    yl=raw_data[i+6]
                    yh=raw_data[i+7]
                    zl=raw_data[i+8]
                    zh=raw_data[i+9]
                    #tzw=raw_data[i+10]
                    jyw=raw_data[i+11]
                    if bin((a1+a2+xl+xh+yl+yh+zl+zh)& 0xff)!=bin(jyw):
                        break
                    if(xh & 0x80):
                        x=0-(65535-((xh << 8) | xl))
                        print("x",x/1000)
                    else:
                        x=(xh << 8) | xl
                        print("x",x/1000)
                    if(yh & 0x80):
                        y=0-(65535-((yh << 8) | yl))
                        print("y",y/1000)
                    else:
                            y=(yh << 8) | yl
                            print("y",y/1000)
                    if(zh & 0x80):
                        z=0-(65535-((zh << 8) | zl))
                        print("z",z/1000)
                    else:
                        z=(zh << 8) | zl
                        print("z",z/1000)
                    twist.linear.x = (x/1000)
                    twist.linear.y =(y/1000)
                    twist.angular.z = (z/1000)*0.95
                    print(twist)
                    velPublisher.publish(twist)
                    break

# 关闭串口
ser.close()
