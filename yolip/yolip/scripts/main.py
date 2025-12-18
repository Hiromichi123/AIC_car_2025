#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from ros2_tools.srv import GarbageClassify

from . import yolo
from . import clip

class yolip_node(Node):
    def __init__(self):
        super().__init__('yolip_node')
        self.bridge = CvBridge()

        # 图像缓存最新3帧
        self.image_cache = deque(maxlen=3)

        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/video',
            self.rgb_cb,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        # 创建垃圾分类服务
        self.classify_service = self.create_service(
            GarbageClassify,
            'garbage_classify',
            self.classify_callback
        )

        # 预热
        self.get_logger().info("开始模型预热...")
        input = np.zeros((960, 960, 3), dtype=np.uint8)
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

    # 垃圾分类服务回调
    def classify_callback(self, request, response):
        """垃圾分类服务 - 对camera1进行 YOLO+CLIP 双重识别"""
        self.get_logger().info("收到垃圾分类请求...")
        
        # 检查缓存中是否有图像
        if not self.image_cache:
            response.success = False
            response.message = "false"
            self.get_logger().error("图像缓存为空，无法进行分类")
            return response
        
        # 获取最新的图像
        latest_frame = self.image_cache[-1]
        rgb_img = latest_frame.copy()
        
        try:
            result = self._perform_classification(rgb_img)

            best = None
            all_candidates = []
            if isinstance(result, dict):
                best = result.get('best')
                all_candidates = result.get('all', []) or []
            else:
                best = result

            # 填充多目标数组字段：每个 YOLO 小框对应一个 best CLIP 结果
            response.categories = [c['category'] for c in all_candidates]
            response.item_names = [c['item_name'] for c in all_candidates]
            response.confidences = [float(c['confidence']) for c in all_candidates]

            if best:
                response.success = True
                response.category = best['category']
                response.item_name = best['item_name']
                response.confidence = float(best['confidence'])

                # 将所有候选结果附带到 message（便于调试/上层查看），不影响字段语义
                if all_candidates:
                    candidates_str = "; ".join(
                        f"[{c['category']}] {c['item_name']}({float(c['confidence']):.2f})"
                        for c in all_candidates
                    )
                    response.message = (
                        f"识别成功: [{best['category']}] {best['item_name']} (置信度: {float(best['confidence']):.2f}). "
                        f"候选: {candidates_str}"
                    )
                else:
                    response.message = (
                        f"识别成功: [{best['category']}] {best['item_name']} (置信度: {float(best['confidence']):.2f})"
                    )

                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "未识别到垃圾物品"
                response.category = ""
                response.item_name = ""
                response.confidence = 0.0

                # 数组字段保持为空
                response.categories = []
                response.item_names = []
                response.confidences = []
                
        except Exception as e:
            self.get_logger().error(f"分类处理异常: {e}")
            response.success = False
            response.message = f"分类失败: {str(e)}"
            response.category = ""
            response.item_name = ""
            response.confidence = 0.0
            response.categories = []
            response.item_names = []
            response.confidences = []
        
        return response

    # 执行分类识别
    def _perform_classification(self, img):
        """进行 YOLO+CLIP 双重识别，返回最佳结果 + 全部候选列表"""
        yolo_results = yolo.infer_cut(img) # YOLO检测
        
        if not yolo_results:
            self.get_logger().warn("YOLO未检测到物体")
            return None
        
        # 对检测到的每个物体进行CLIP分类
        best_result = None
        best_confidence = 0.0
        all_results = []
        
        for cut_result in yolo_results:
            if not cut_result:
                continue

            clip_results = clip.infer(cut_result.square_cut_img)
            
            # 获取最佳CLIP结果
            clip_best = clip.get_best_result(clip_results, min_confidence=0.3)
            if not clip_best:
                continue

            candidate = {
                'category': clip_best.category, # 类别
                'item_name': clip_best.item_name, # 物品名称
                'confidence': float(clip_best.confidence) # 置信度
            }
            all_results.append(candidate)
            
            # 保存置信度最高的结果
            if clip_best.confidence > best_confidence:
                best_confidence = clip_best.confidence
                best_result = candidate

        return {
            'best': best_result,
            'all': all_results,
        }

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
