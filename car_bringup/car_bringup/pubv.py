#!/usr/bin/env python3
#coding=utf-8
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelRawPublisher(Node):
    def __init__(self):
        super().__init__('vel_raw_pub')
        self.velPublisher = self.create_publisher(Twist, "/vel_raw", 100)
        self.ser = serial.Serial("/dev/carserial", 115200, timeout=5)
        if self.ser.is_open:
            self.get_logger().info('Serial port opened successfully')
        
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
    
    def timer_callback(self):
        raw_data = self.ser.read(12)
        if len(raw_data) > 0:
            for i in range(len(raw_data) - 1):
                if raw_data[i] == 0xAA and raw_data[i+1] == 0xBB:
                    if i != 0:
                        break
                    twist = Twist()
                    # 检查字节流中是否包含协议数据头
                    a1 = raw_data[i+2]
                    a2 = raw_data[i+3]
                    xl = raw_data[i+4]
                    xh = raw_data[i+5]
                    yl = raw_data[i+6]
                    yh = raw_data[i+7]
                    zl = raw_data[i+8]
                    zh = raw_data[i+9]
                    jyw = raw_data[i+11]
                    
                    if bin((a1+a2+xl+xh+yl+yh+zl+zh) & 0xff) != bin(jyw):
                        break
                    
                    if (xh & 0x80):
                        x = 0 - (65535 - ((xh << 8) | xl))
                        self.get_logger().info(f"x: {x/1000}")
                    else:
                        x = (xh << 8) | xl
                        self.get_logger().info(f"x: {x/1000}")
                    
                    if (yh & 0x80):
                        y = 0 - (65535 - ((yh << 8) | yl))
                        self.get_logger().info(f"y: {y/1000}")
                    else:
                        y = (yh << 8) | yl
                        self.get_logger().info(f"y: {y/1000}")
                    
                    if (zh & 0x80):
                        z = 0 - (65535 - ((zh << 8) | zl))
                        self.get_logger().info(f"z: {z/1000}")
                    else:
                        z = (zh << 8) | zl
                        self.get_logger().info(f"z: {z/1000}")
                    
                    twist.linear.x = (x/1000)
                    twist.linear.y = (y/1000)
                    twist.angular.z = (z/1000) * 0.95
                    self.get_logger().info(f'{twist}')
                    self.velPublisher.publish(twist)
                    break
    
    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = VelRawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
