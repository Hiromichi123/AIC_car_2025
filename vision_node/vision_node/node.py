import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys
from ultralytics import YOLO as YOLOModel # type: ignore
from PIL import ImageFont, ImageDraw, Image as PILImage

from ros2_tools.srv import YOLO
from ros2_tools.srv import OCR

# 添加PaddleOCR路径到sys.path - 使用绝对路径
VISION_NODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PADDLEOCR_DIR = os.path.join(VISION_NODE_DIR, 'ocr')

# 确保路径存在且添加到 sys.path
if os.path.exists(PADDLEOCR_DIR):
    if PADDLEOCR_DIR not in sys.path:
        sys.path.insert(0, PADDLEOCR_DIR)
    # 同时添加 tools 目录
    tools_dir = os.path.join(PADDLEOCR_DIR, 'tools')
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
else:
    raise RuntimeError(f"PaddleOCR directory not found: {PADDLEOCR_DIR}")

# 导入PaddleOCR
try:
    from paddleocr import PaddleOCR
except ImportError as e:
    raise ImportError(f"Failed to import PaddleOCR: {e}\nPaddleOCR directory: {PADDLEOCR_DIR}")

# 导入配置
from .config import (
    CURRENT_MODEL, OCR_SAVE_DIR, 
    YOLO_MODEL_PATH, YOLO_FONT_PATH, YOLO_SAVE_DIR, YOLO_LABELS
)

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 初始化CvBridge用于ROS图像消息转换
        self.bridge = CvBridge()
        
        # 存储最新的图像数据
        self.camera1_image = None
        self.camera1_info = None
        self.camera2_image = None
        self.camera2_info = None
        
        # 创建订阅者
        self.camera1_image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.camera1_image_callback,
            10
        )
        self.camera1_info_sub = self.create_subscription(
            CameraInfo,
            '/camera1/camera_info',
            self.camera1_info_callback,
            10
        )
        self.camera2_image_sub = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.camera2_image_callback,
            10
        )
        self.camera2_info_sub = self.create_subscription(
            CameraInfo,
            '/camera2/camera_info',
            self.camera2_info_callback,
            10
        )
        
        # 创建服务
        self.srv_yolo = self.create_service(YOLO, 'yolo_trigger', self.yolo_callback)
        self.srv_ocr = self.create_service(OCR, 'ocr_trigger', self.ocr_callback)
        
        # 初始化YOLO模型 - 使用配置文件
        self.yolo_model_path = YOLO_MODEL_PATH
        self.yolo_font_path = YOLO_FONT_PATH
        self.yolo_save_dir = YOLO_SAVE_DIR
        os.makedirs(self.yolo_save_dir, exist_ok=True)
        
        try:
            self.yolo_model = YOLOModel(self.yolo_model_path)
            self.get_logger().info("YOLO模型加载成功")
        except Exception as e:
            self.get_logger().error(f"YOLO模型加载失败: {e}")
            self.yolo_model = None
        
        # 自定义YOLO标签 - 使用配置文件
        self.custom_labels = YOLO_LABELS
        
        # 初始化PaddleOCR - 使用配置文件中的模型
        try:
            self.ocr_engine = PaddleOCR(**CURRENT_MODEL, show_log=False)
            self.get_logger().info(f"PaddleOCR引擎加载成功 - 使用模型:")
            self.get_logger().info(f"  检测模型: {CURRENT_MODEL['det_model_dir']}")
            self.get_logger().info(f"  识别模型: {CURRENT_MODEL['rec_model_dir']}")
        except Exception as e:
            self.get_logger().error(f"PaddleOCR引擎加载失败: {e}")
            self.ocr_engine = None
        
        self.ocr_save_dir = OCR_SAVE_DIR
        os.makedirs(self.ocr_save_dir, exist_ok=True)
        
        self.get_logger().info("VisionNode初始化完成")
    
    # Camera1 回调函数
    def camera1_image_callback(self, msg):
        try:
            self.camera1_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Camera1图像转换失败: {e}")
    
    def camera1_info_callback(self, msg):
        self.camera1_info = msg
    
    # Camera2 回调函数
    def camera2_image_callback(self, msg):
        try:
            self.camera2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Camera2图像转换失败: {e}")
    
    def camera2_info_callback(self, msg):
        self.camera2_info = msg
    
    def yolo_callback(self, request, response):
        """YOLO检测服务回调函数"""
        self.get_logger().info("开始YOLO检测...")
        
        if self.yolo_model is None:
            response.success = False
            response.message = "YOLO模型未加载"
            return response
        
        # 使用camera1的图像进行YOLO检测
        if self.camera2_image is None:
            response.success = False
            response.message = "Camera1图像未接收"
            return response
        
        try:
            frame = self.camera2_image.copy()
            
            # YOLO检测
            results = self.yolo_model(frame)
            boxes = results[0].boxes
            
            # 转换为PIL图像以绘制中文
            img_pil = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(img_pil)
            
            try:
                font = ImageFont.truetype(self.yolo_font_path, 28, encoding="utf-8")
            except:
                font = ImageFont.load_default()
            
            detection_results = []
            for box in boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)
                label = self.custom_labels.get(cls_id, f"未知类别({cls_id})")
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                color = (0, 255, 0) if cls_id == 1 else (255, 0, 0)
                
                # 绘制框
                draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
                
                # 绘制标签
                text = f"{label} {conf:.2f}"
                draw.text((x1, y1 - 30), text, font=font, fill=color)
                
                detection_results.append(f"{label} (置信度: {conf:.2f})")
                self.get_logger().info(f"检测到: {label} ({conf:.2f})")
            
            # 转回OpenCV格式
            frame = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
            
            # 保存结果
            import time
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            result_path = os.path.join(self.yolo_save_dir, f"yolo_result_{timestamp}.jpg")
            cv2.imwrite(result_path, frame)
            
            response.success = True
            response.message = f"YOLO检测完成，检测到{len(boxes)}个目标。结果: {', '.join(detection_results)}"
            
        except Exception as e:
            self.get_logger().error(f"YOLO检测失败: {e}")
            response.success = False
            response.message = f"YOLO检测失败: {str(e)}"
        
        return response
    
    def ocr_callback(self, request, response):
        """OCR识别服务回调函数"""
        self.get_logger().info("开始OCR识别...")
        
        if self.ocr_engine is None:
            response.success = False
            response.message = "OCR引擎未加载"
            return response
        
        # 使用camera2的图像进行OCR识别
        if self.camera1_image is None:
            response.success = False
            response.message = "Camera2图像未接收"
            return response
        
        try:
            frame = self.camera1_image.copy()
            
            # OCR识别
            result = self.ocr_engine.ocr(frame, cls=True)
            
            ocr_results = []
            if result and result[0]:
                for line in result[0]:
                    text = line[1][0]
                    confidence = line[1][1]
                    ocr_results.append(f"{text} (置信度: {confidence:.2f})")
                    self.get_logger().info(f"识别到文本: {text} (置信度: {confidence:.2f})")
            
            # 保存结果（可选）
            import time
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            result_path = os.path.join(self.ocr_save_dir, f"ocr_result_{timestamp}.jpg")
            cv2.imwrite(result_path, frame)
            
            response.success = True
            if ocr_results:
                response.message = f"OCR识别完成，识别到{len(ocr_results)}条文本。结果: {', '.join(ocr_results)}"
            else:
                response.message = "OCR识别完成，未识别到文本"
            
        except Exception as e:
            self.get_logger().error(f"OCR识别失败: {e}")
            response.success = False
            response.message = f"OCR识别失败: {str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
