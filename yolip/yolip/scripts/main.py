#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from . import yolo
from . import clip

class yolip_node(Node):
    def __init__(self):
        # 动物字典映射
        self.OBJECT_TYPE_MAPPING = {
            '大象': 1,
            '老虎': 2,
            '狼': 3,
            '猴子': 4,
            '孔雀': 5
        }

        super().__init__('yolip_node')
        self.bridge = CvBridge()

        # 图像缓存最新3帧
        self.image_cache = deque(maxlen=3)
        self.odom_msg = None

        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_cb,
            qos_profile
        )

        # 预热
        self.get_logger().info("开始模型预热...")
        input = np.zeros((960, 960, 3), dtype=np.uint8) # 空图像
        yolo.infer_cut(input)
        clip.infer(input)
        self.get_logger().info("yolip_node初始化完成")
        rclpy.spin_once(self, timeout_sec=0.1)

    # rgb回调
    def rgb_cb(self, msg):
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 添加到缓存（自动保持最新3帧）
            self.image_cache.append(bgr_img.copy())
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")

    # 区域触发回调
    def region_cb(self, msg):
        if msg.data == 0:
            return
 
        # 检查缓存中是否有图像
        if not self.image_cache:
            self.get_logger().warn("触发识别但图像缓存为空")
            return
        
        # 获取最新的图像
        latest_frame = self.image_cache[-1]
        rgb_img = latest_frame.copy()
        try:
            img_copy = rgb_img.copy()
            self.classify(rgb_img, img_copy, msg)

        except Exception as e:
            self.get_logger().error(f"识别处理异常: {e}")

    # 进行双重识别
    def classify(self, img, img_copy, msg):
        current_results = yolo.infer_cut(img) # 进行yolo推理

        yolo.draw_all_yolo_boxes(img_copy, current_results) # 绘制yolo矩形框

        #对当前区域的yolo框进行CLIP识别
        valid_results = []
        for cut_result in current_results:
            if not cut_result:
                continue

            clip_results = clip.infer(cut_result.square_cut_img)
            
            probs = {}
            for clip_result in clip_results:
                if clip_result.confidence > 0.3:
                    probs[clip_result.name] = clip_result.confidence
            if not probs:
                continue

            result = max(probs, key=probs.get)
            confidence = probs[result]
            center_x, center_y = cut_result.get_center_point()
            valid_results.append({
                'cut_result': cut_result,
                'result': result,
                'confidence': confidence,
                'center_x': center_x,
                'center_y': center_y,
                'probs': probs
            })

def main(args=None):
    rclpy.init(args=args)
    node = yolip_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("yolip_node被中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
