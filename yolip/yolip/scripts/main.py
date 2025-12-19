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

from .  import yolo
from .  import clip

class yolip_node(Node):
    def __init__(self):
        super().__init__('yolip_node')
        self.bridge = CvBridge()

        # 图像缓存最新3帧
        self.image_cache = deque(maxlen=3)

        # 'yolip' 或 'yolo_cls'
        self.classification_mode = 'yolo_cls'

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
            self. classify_callback
        )

        # 预热
        self.get_logger().info("开始模型预热...")
        input_img = np.zeros((960, 960, 3), dtype=np.uint8)
        
        if self.classification_mode == 'yolip':
            yolo.infer_cut(input_img)
            clip.infer(input_img)
            self.get_logger().info("YOLO+CLIP 模式预热完成")
        elif self.classification_mode == 'yolo_cls':
            if yolo.model_cls is not None:
                yolo.infer_classify(input_img)
                self.get_logger().info("YOLO-CLS 模式预热完成")
            else:
                self.get_logger().error("YOLO分类模型未加载，将回退到YOLIP模式")
                self.classification_mode = 'yolip'
        
        self.get_logger().info("yolip_node初始化完成")
        rclpy.spin_once(self, timeout_sec=0.1)

    # rgb回调
    def rgb_cb(self, msg):
        try:
            bgr_img = self.bridge. imgmsg_to_cv2(msg, "bgr8")
            self.image_cache.append(bgr_img.copy())
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")

    # 垃圾分类服务回调
    def classify_callback(self, request, response):
        """垃圾分类服务 - 支持YOLO+CLIP或纯YOLO-CLS"""
        self.get_logger().info(f"收到垃圾分类请求（模式: {self.classification_mode}）...")
        
        if not self.image_cache:
            response.success = False
            response.message = "图像缓存为空"
            self.get_logger().error("图像缓存为空，无法进行分类")
            return response
        
        latest_frame = self.image_cache[-1]
        rgb_img = latest_frame.copy()
        
        try:
            # ========== 根据模式选择识别方法 ==========
            if self. classification_mode == 'yolo_cls':
                result = self._perform_yolo_cls_classification(rgb_img)
            else:  # 默认 'yolip'
                result = self._perform_yolip_classification(rgb_img)

            best = None
            all_candidates = []
            if isinstance(result, dict):
                best = result. get('best')
                all_candidates = result. get('all', []) or []
            else:
                best = result

            # 填充多目标数组字段
            response.categories = [c['category'] for c in all_candidates]
            response.item_names = [c['item_name'] for c in all_candidates]
            response.confidences = [float(c['confidence']) for c in all_candidates]

            if best and all_candidates:
                response.success = True
                response.category = best['category']
                response.item_name = best['item_name']
                response.confidence = float(best['confidence'])

                if len(all_candidates) > 1:
                    items_str = ", ".join(
                        f"[{c['category']}] {c['item_name']}({float(c['confidence']):.2f})"
                        for c in all_candidates
                    )
                    response.message = f"检测到 {len(all_candidates)} 个物品（从左到右）:  {items_str}"
                else:
                    response.message = (
                        f"检测到 [{best['category']}] {best['item_name']} "
                        f"(置信度: {float(best['confidence']):.2f})"
                    )

                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "未识别到垃圾物品"
                response.category = ""
                response.item_name = ""
                response.confidence = 0.0
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

    # ========== 原方法：YOLO+CLIP 双重识别 ==========
    def _perform_yolip_classification(self, img):
        """进行 YOLO+CLIP 双重识别，返回最佳结果 + 全部候选列表（按从左到右排序）"""
        yolo_results = yolo.infer_cut(img, confidence_threshold=0.3, sort_left_to_right=True)
        
        if not yolo_results:
            self.get_logger().warn("YOLO未检测到物体")
            return None
        
        self.get_logger().info(f"YOLO检测到 {len(yolo_results)} 个物体（已按从左到右排序）")
        
        best_result = None
        best_confidence = 0.0
        all_results = []
        
        for idx, cut_result in enumerate(yolo_results):
            if not cut_result: 
                continue

            center_x, center_y = cut_result.get_center_point()
            clip_results = clip.infer(cut_result. square_cut_img)
            
            clip_best = clip. get_best_result(clip_results, min_confidence=0.3)
            if not clip_best:
                self.get_logger().warn(f"第{idx+1}个目标CLIP识别置信度过低，跳过")
                continue

            candidate = {
                'category': clip_best.category,
                'item_name': clip_best.item_name,
                'confidence': float(clip_best.confidence),
                'position_index': idx,
                'center_x': center_x
            }
            all_results.append(candidate)
            
            if clip_best.confidence > best_confidence:
                best_confidence = clip_best.confidence
                best_result = candidate

        return {
            'best': best_result,
            'all': all_results,
        }

    # ========== 纯YOLO分类识别 ==========
    def _perform_yolo_cls_classification(self, img):
        """使用YOLO分类模型直接识别垃圾（36小类+4大类）"""
        cls_results = yolo. infer_classify(img, confidence_threshold=0.3, sort_left_to_right=True)
        
        if not cls_results:
            self.get_logger().warn("YOLO分类模型未检测到物体")
            return None
        
        self.get_logger().info(f"YOLO-CLS检测到 {len(cls_results)} 个物体（已按从左到右排序）")
        
        best_result = None
        best_confidence = 0.0
        all_results = []
        
        for idx, cls_result in enumerate(cls_results):
            candidate = {
                'category': cls_result.category,      # 4大类
                'item_name':  cls_result.item_name,    # 36小类
                'confidence': float(cls_result.confidence),
                'position_index': idx,
                'center_x': cls_result.center_x
            }
            all_results.append(candidate)
            
            if cls_result. confidence > best_confidence:
                best_confidence = cls_result. confidence
                best_result = candidate

        return {
            'best':  best_result,
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
