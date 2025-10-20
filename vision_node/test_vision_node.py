#!/usr/bin/env python3
"""
测试Vision Node的YOLO和OCR服务
用于验证双摄像头的同时识别功能
"""

import rclpy
from rclpy.node import Node
from ros2_tools.srv import YOLO, OCR
import sys

class VisionTestClient(Node):
    def __init__(self):
        super().__init__('vision_test_client')
        
        # 创建YOLO服务客户端
        self.yolo_client = self.create_client(YOLO, 'yolo_trigger')
        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待YOLO服务...')
        
        # 创建OCR服务客户端
        self.ocr_client = self.create_client(OCR, 'ocr_trigger')
        while not self.ocr_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待OCR服务...')
        
        self.get_logger().info('服务连接成功！')
    
    def test_yolo(self):
        """测试YOLO检测（camera1 + camera2）"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始测试YOLO检测（双摄像头）...')
        self.get_logger().info('=' * 60)
        
        request = YOLO.Request()
        future = self.yolo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f'✅ YOLO检测成功!')
            self.get_logger().info(f'结果: {result.message}')
        else:
            self.get_logger().error(f'❌ YOLO检测失败: {result.message}')
        
        return result.success
    
    def test_ocr(self):
        """测试OCR识别（camera1 + camera2）"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始测试OCR识别（双摄像头）...')
        self.get_logger().info('=' * 60)
        
        request = OCR.Request()
        future = self.ocr_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f'✅ OCR识别成功!')
            self.get_logger().info(f'结果: {result.message}')
        else:
            self.get_logger().error(f'❌ OCR识别失败: {result.message}')
        
        return result.success

def main(args=None):
    rclpy.init(args=args)
    client = VisionTestClient()
    
    try:
        # 测试YOLO
        yolo_success = client.test_yolo()
        
        # 等待一下
        import time
        time.sleep(1)
        
        # 测试OCR
        ocr_success = client.test_ocr()
        
        # 打印总结
        print('\n' + '=' * 60)
        print('测试总结:')
        print(f'  YOLO检测: {"✅ 成功" if yolo_success else "❌ 失败"}')
        print(f'  OCR识别:  {"✅ 成功" if ocr_success else "❌ 失败"}')
        print('=' * 60)
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
