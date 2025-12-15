import cv2
import os
import sys
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from importlib import import_module
from ultralytics import YOLO as YOLOModel  # type: ignore
from PIL import ImageFont, ImageDraw, Image as PILImage

from ros2_tools.srv import YOLO
from ros2_tools.srv import OCR
from ros2_tools.srv import GarbageClassify

from . import config


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        # 允许通过 ROS 参数覆盖资源路径
        self.declare_parameter("vision_node_src_dir", config.VISION_NODE_SRC_DIR)
        requested_src_dir = (
            self.get_parameter("vision_node_src_dir").get_parameter_value().string_value
        )
        try:
            resolved_src_dir = config.configure_paths(requested_src_dir)
            self.get_logger().info(f"使用 vision_node 资源目录: {resolved_src_dir}")
        except ValueError as exc:
            self.get_logger().warn(f"指定的 vision_node 目录无效，回退到默认值: {exc}")
            resolved_src_dir = config.VISION_NODE_SRC_DIR

        # 确保 PaddleOCR 依赖路径在 sys.path 中
        self._ensure_paddleocr_paths(resolved_src_dir)

        # 动态导入 PaddleOCR，确保在路径更新后加载
        try:
            paddleocr_module = import_module("paddleocr")
            self._paddleocr_ctor = paddleocr_module.PaddleOCR
        except Exception as exc:
            raise ImportError(
                f"无法导入 PaddleOCR，请检查目录 {resolved_src_dir}/ocr: {exc}"
            ) from exc

        # 初始化CvBridge用于ROS图像消息转换
        self.bridge = CvBridge()

        # 存储最新的图像数据
        self.camera1_image = None  # 单目旋转相机
        self.camera2_image = None  # 双目固定相机

        # 使用 ReentrantCallbackGroup 允许在服务回调中处理图像订阅
        self.cb_group = ReentrantCallbackGroup()

        # 创建订阅者
        # 相机话题通常使用 sensor data QoS，避免可靠模式导致丢帧阻塞
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # 普通相机
        self.camera1_image_sub = self.create_subscription(
            Image, "/camera/video", self.camera1_image_callback, qos_profile, callback_group=self.cb_group
        )
        # 深度相机
        self.camera2_image_sub = self.create_subscription(
            Image, "/camera/d435/color/image_raw", self.camera2_image_callback, qos_profile, callback_group=self.cb_group
        )
        self.srv_yolo = self.create_service(YOLO, "yolo_trigger", self.yolo_callback, callback_group=self.cb_group)
        self.srv_ocr = self.create_service(OCR, "ocr_trigger", self.ocr_callback, callback_group=self.cb_group)

        # 创建垃圾分类客户端
        self.garbage_classify_client = self.create_client(GarbageClassify, 'garbage_classify')

        # 初始化YOLO模型
        self.yolo_model_path_default = config.YOLO_MODEL_PATH
        self.yolo_font_path = config.YOLO_FONT_PATH
        self.yolo_save_dir = "/home/jetson/ros2/AIC_car_2025/vision_node/yolo/result"
        os.makedirs(self.yolo_save_dir, exist_ok=True)

        self.yolo_models = {}  # 缓存已加载模型
        self.yolo_model_paths = config.YOLO_MODELS

        # YOLO模型标签配置 - 按模型名称分类
        self.yolo_labels_config = config.YOLO_LABELS

        # Custom maps
        self.custom_color_map = {
            0: (255, 0, 0),     # 社区/人类类目 1
            1: (0, 255, 0),     # 社区/人类类目 2
            2: (0, 0, 255),     # 红灯
            3: (0, 255, 255),   # 绿灯
            4: (255, 255, 0),   # 黄灯
        }
        self.custom_label_map = {
            0: "target_1",
            1: "target_2",
            2: "红灯",
            3: "绿灯",
            4: "黄灯",
        }

        # 初始化PaddleOCR
        try:
            self.ocr_engine = self._paddleocr_ctor(**config.CURRENT_MODEL, show_log=False)
            self.get_logger().info("PaddleOCR引擎加载成功")
        except Exception as e:
            self.get_logger().error(f"PaddleOCR引擎加载失败: {e}")
            self.ocr_engine = None

        self.ocr_save_dir = config.OCR_SAVE_DIR
        os.makedirs(self.ocr_save_dir, exist_ok=True)

        self.get_logger().info("VisionNode初始化完成")

    def _ensure_paddleocr_paths(self, src_dir: str) -> None:
        """将 OCR 依赖目录注入 sys.path，便于按需覆盖。"""
        ocr_dir = os.path.join(src_dir, "ocr")
        tools_dir = os.path.join(ocr_dir, "tools")

        for path in (ocr_dir, tools_dir):
            if os.path.isdir(path) and path not in sys.path:
                sys.path.insert(0, path)

        if not os.path.isdir(ocr_dir):
            raise RuntimeError(f"未找到 PaddleOCR 目录: {ocr_dir}")

    # Camera1 回调
    def camera1_image_callback(self, msg):
        try:
            self.camera1_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera1图像转换失败: {e}")

    # Camera2 回调
    def camera2_image_callback(self, msg):
        try:
            self.camera2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera2图像转换失败: {e}")

    def yolo_callback(self, request, response):
        """YOLO检测服务回调函数 - 支持指定模型与相机"""
        model_name = request.model.strip()
        camera_mode = (request.camera or "").strip().lower() or "both"
        
        # 验证相机参数
        if camera_mode not in ("camera1", "camera2", "both"):
            response.success = False
            response.message = f"不支持的camera参数: {request.camera}"
            return response

        # 加载指定模型
        model, model_path = self._get_yolo_model(model_name)
        if model is None:
            response.success = False
            response.message = f"YOLO模型未加载: {model_name or 'default'}"
            return response

        try:
            # 交通灯模型循环检测直到绿灯
            if model_name == "traffic_light":
                return self._handle_traffic_light_detection(request, response, model, model_path, model_name, camera_mode)
            
            # 垃圾桶检测 - 检测到后调用 yolip 服务
            if model_name == "rubbish_bin":
                return self._handle_rubbish_bin_detection(request, response, model, model_path, model_name, camera_mode)
            
            # 默认检测
            return self._handle_standard_detection(request, response, model, model_path, model_name, camera_mode)

        except Exception as e:
            self.get_logger().error(f"YOLO检测异常: {e}")
            response.success = False
            response.message = f"YOLO检测失败: {str(e)}"
            return response

    def _handle_traffic_light_detection(self, request, response, model, model_path, model_name, camera_mode):
        """交通灯检测 - 循环直到绿灯或超时"""
        max_wait = 20.0
        poll_interval = 0.5
        deadline = time.time() + max_wait

        while time.time() < deadline:
            ts = time.strftime("%Y%m%d-%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}"
            
            # 获取当前相机图像
            current_images = self._get_camera_images(camera_mode)
            if not current_images:
                # 图像未就绪，继续等待
                time.sleep(poll_interval)
                continue

            # 执行检测
            all_results = []
            for cam_name, frame in current_images:
                det_results, _ = self._process_yolo_image(
                    frame.copy(),
                    f"{cam_name}_{ts}",
                    cam_name,
                    model,
                    model_name,
                    suppress_empty_warn=True,
                    save_result=False,
                    return_frame=True,
                )
                all_results.extend(det_results)

            # 检查是否检测到绿灯
            green_hits = [msg for msg in all_results if "绿灯" in msg]
            if green_hits:
                response.success = True
                response.message = f"检测到绿灯: {'; '.join(green_hits)}"
                self.get_logger().info("✅ 绿灯检测成功")
                return response

            time.sleep(poll_interval)

        # 超时未检测到绿灯
        response.success = False
        response.message = f"等待绿灯超时({max_wait}秒)"
        self.get_logger().warn("⚠️ 绿灯检测超时")
        return response

    def _handle_rubbish_bin_detection(self, request, response, model, model_path, model_name, camera_mode):
        """垃圾桶检测 - 检测到垃圾桶后调用yolip服务进行垃圾分类"""
        ts = time.strftime("%Y%m%d-%H%M%S")
        
        # 获取相机图像
        camera_images = self._get_camera_images(camera_mode)
        if not camera_images:
            response.success = False
            response.message = self._get_missing_camera_message(camera_mode)
            return response

        # 使用垃圾桶检测模型检测垃圾桶
        all_results = []
        for cam_name, frame in camera_images:
            self.get_logger().info(f"检测垃圾桶 - {cam_name}...")
            det_results = self._process_yolo_image(
                frame.copy(),
                f"{cam_name}_{ts}_rubbish_bin",
                cam_name,
                model,
                model_name,
                suppress_empty_warn=False,
                save_result=True,
                return_frame=False,
            )
            all_results.extend(det_results)

        if not all_results:
            response.success = False
            response.message = "未检测到垃圾桶"
            return response

        # 检测到垃圾桶后，调用yolip服务进行垃圾分类
        self.get_logger().info("检测到垃圾桶，调用yolip服务进行垃圾分类...")
        
        # 等待服务可用
        if not self.garbage_classify_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = f"yolip垃圾分类服务不可用。垃圾桶检测结果: {'; '.join(all_results)}"
            self.get_logger().error("yolip服务未启动")
            return response

        # 调用垃圾分类服务
        classify_request = GarbageClassify.Request()
        future = self.garbage_classify_client.call_async(classify_request)
        
        # 阻塞等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            classify_response = future.result()
            if classify_response.success:
                response.success = True
                response.message = (
                    f"垃圾分类完成: [{classify_response.category}] "
                    f"{classify_response.item_name} "
                    f"(置信度: {classify_response.confidence:.2f}). "
                    f"垃圾桶位置: {'; '.join(all_results)}"
                )
                self.get_logger().info(f"✅ {response.message}")
            else:
                response.success = False
                response.message = f"垃圾分类失败: {classify_response.message}. 垃圾桶检测: {'; '.join(all_results)}"
                self.get_logger().warn(f"⚠️ {response.message}")
        else:
            response.success = False
            response.message = f"垃圾分类服务调用超时。垃圾桶检测: {'; '.join(all_results)}"
            self.get_logger().error("yolip服务调用超时")
        
        return response

    def _handle_standard_detection(self, request, response, model, model_path, model_name, camera_mode):
        """处理标准一次性检测"""
        ts = time.strftime("%Y%m%d-%H%M%S")
        
        # 获取相机图像
        camera_images = self._get_camera_images(camera_mode)
        if not camera_images:
            response.success = False
            response.message = self._get_missing_camera_message(camera_mode)
            return response

        # 执行检测
        all_results = []
        for cam_name, frame in camera_images:
            self.get_logger().info(f"处理{cam_name}图像... 使用模型: {model_path}")
            det_results = self._process_yolo_image(
                frame.copy(),
                f"{cam_name}_{ts}",
                cam_name,
                model,
                model_name,
                suppress_empty_warn=False,
                save_result=True,
                return_frame=False,
            )
            all_results.extend(det_results)

        # 返回结果
        response.success = True
        if all_results:
            response.message = f"YOLO检测到{len(all_results)}个目标。结果: {'; '.join(all_results)}"
        else:
            response.message = "YOLO未检测到目标"
        
        return response

    def _get_camera_images(self, camera_mode):
        """获取指定相机的当前图像，返回 [(camera_name, frame), ...] 列表"""
        images = []
        
        if camera_mode in ("camera1", "both"):
            if self.camera1_image is not None:
                images.append(("Camera1", self.camera1_image))
        
        if camera_mode in ("camera2", "both"):
            if self.camera2_image is not None:
                images.append(("Camera2", self.camera2_image))
        
        return images

    def _get_missing_camera_message(self, camera_mode):
        """生成缺失相机的错误消息"""
        missing = []
        if camera_mode in ("camera1", "both") and self.camera1_image is None:
            missing.append("Camera1")
        if camera_mode in ("camera2", "both") and self.camera2_image is None:
            missing.append("Camera2")
        
        if missing:
            return f"{'、'.join(missing)}图像未接收"
        return "未接收到可用图像"

    def _process_yolo_image(
        self,
        frame,
        filename_prefix,
        camera_name,
        model,
        model_name,
        suppress_empty_warn: bool = False,
        save_result: bool = True,
        return_frame: bool = False,
    ):
        """处理单张图像的YOLO检测

        Args:
            frame: 输入图像
            filename_prefix: 保存文件名前缀
            camera_name: 相机名称
            model: YOLO模型实例
            model_name: 模型名称,用于查找对应的标签配置
            suppress_empty_warn: 是否抑制空结果警告
            save_result: 是否保存结果图像
            return_frame: 是否返回处理后的图像帧
        """
        detection_results = []

        # YOLO检测，输入尺寸960
        results = model(frame, imgsz=960)
        
        # 准备绘制
        img_pil = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        
        # 加载字体
        try:
            font = ImageFont.truetype(self.yolo_font_path, 28, encoding="utf-8")
        except:
            font = ImageFont.load_default()

        has_detections = False
        
        # 处理目标检测结果 (Boxes)
        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            has_detections = True
            boxes = results[0].boxes

            # 获取当前模型的标签和颜色配置
            model_config = self.yolo_labels_config.get(
                model_name, self.yolo_labels_config["default"]
            )
            label_map = model_config.get("labels", {})
            color_map = model_config.get("colors", {})

            # 处理每个检测框
            for box in boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)

                # 优先使用自定义映射
                if cls_id in self.custom_label_map:
                    label = self.custom_label_map[cls_id]
                    color = self.custom_color_map.get(cls_id, (255, 255, 255))
                else:
                    label = label_map.get(cls_id, f"未知类别({cls_id})")
                    color = color_map.get(cls_id, (255, 255, 255))

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 绘制矩形框
                draw.rectangle([x1, y1, x2, y2], outline=color, width=3)

                # 绘制中文标签
                text = f"{label} {conf:.2f}"
                draw.text((x1, y1 - 30), text, font=font, fill=color)

                # 记录结果
                result_str = f"[{camera_name}] {label} (置信度: {conf:.2f})"
                detection_results.append(result_str)
        
        # 2. 处理图像分类结果 (Probs) - 针对交通灯分类模型
        elif results and hasattr(results[0], 'probs') and results[0].probs is not None:
            has_detections = True
            probs = results[0].probs
            # 获取置信度最高的类别
            top1_index = int(probs.top1)
            conf = float(probs.top1conf)
            
            # 获取类别名称
            label = results[0].names[top1_index] if results[0].names else str(top1_index)
            
            # 映射英文标签到中文，以便后续逻辑判断
            display_label = label
            if "green" in label.lower():
                display_label = "绿灯"
            elif "red" in label.lower():
                display_label = "红灯"
            elif "yellow" in label.lower():
                display_label = "黄灯"
            
            result_str = f"[{camera_name}] {display_label} (置信度: {conf:.2f})"
            detection_results.append(result_str)
            
            # 在左上角绘制分类结果
            draw.text((30, 30), f"Type: {display_label} {conf:.2f}", font=font, fill=(0, 255, 0))

        # 转回OpenCV格式
        frame = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

        # 保存检测结果图像
        if save_result:
            result_path = os.path.join(
                self.yolo_save_dir, f"{filename_prefix}_result.jpg"
            )
            cv2.imwrite(result_path, frame)
            self.get_logger().info(f"✅ {camera_name}检测结果已保存到: {result_path}")
            
        if not has_detections:
             if not suppress_empty_warn:
                self.get_logger().info(f"{camera_name} 未检测到目标")

        if return_frame:
            return detection_results, frame
        return detection_results

    def _get_yolo_model(self, model_name: str):
        """按需加载或复用 YOLO 模型，返回 (model, path)"""
        chosen_path = None

        if model_name:
            chosen_path = self.yolo_model_paths.get(model_name)
            if chosen_path is None and os.path.isfile(model_name):
                chosen_path = model_name
        else:
            chosen_path = self.yolo_model_path_default

        if chosen_path is None:
            self.get_logger().error(f"未找到指定的 YOLO 模型: {model_name}")
            return None, None

        for _, cached in self.yolo_models.items():
            if cached[1] == chosen_path:
                return cached

        try:
            model = YOLOModel(chosen_path)
            key = model_name or "default"
            self.yolo_models[key] = (model, chosen_path)
            self.get_logger().info(f"加载 YOLO 模型: {chosen_path}")
            return model, chosen_path
        except Exception as exc:
            self.get_logger().error(f"加载 YOLO 模型失败 {chosen_path}: {exc}")
            return None, None

    def ocr_callback(self, request, response):
        """OCR识别服务回调函数 - 同时处理camera1和camera2"""
        self.get_logger().info("开始OCR识别...")

        if self.ocr_engine is None:
            response.success = False
            response.message = "OCR引擎未加载"
            return response

        # 检查两个相机的图像
        if self.camera1_image is None and self.camera2_image is None:
            response.success = False
            response.message = "Camera1和Camera2图像均未接收"
            return response

        try:
            import time

            timestamp = time.strftime("%Y%m%d-%H%M%S")
            all_ocr_results = []

            # 处理Camera1
            if self.camera1_image is not None:
                camera1_results = self._process_ocr_image(
                    self.camera1_image.copy(), f"camera1_{timestamp}", "Camera1"
                )
                all_ocr_results.extend(camera1_results)

            # 处理Camera2
            if self.camera2_image is not None:
                camera2_results = self._process_ocr_image(
                    self.camera2_image.copy(), f"camera2_{timestamp}", "Camera2"
                )
                all_ocr_results.extend(camera2_results)

            response.success = True
            if all_ocr_results:
                response.message = f"OCR识别到{len(all_ocr_results)}条文本。结果:{'; '.join(all_ocr_results)}"
            else:
                response.message = "OCR未识别到文本"

        except Exception as e:
            response.success = False
            response.message = f"OCR识别失败:{str(e)}"

        return response

    def _process_ocr_image(self, frame, filename_prefix, camera_name):
        """处理单张图像的OCR识别"""
        ocr_results = []

        # OCR识别
        result = self.ocr_engine.ocr(frame, cls=True)

        if result and result[0]:
            for line in result[0]:
                text = line[1][0]
                confidence = line[1][1]
                result_str = f"[{camera_name}] {text} (置信度:{confidence:.2f})"
                ocr_results.append(result_str)
                self.get_logger().info(f"识别到文本:{result_str}")

        # 保存结果图像
        result_path = os.path.join(self.ocr_save_dir, f"{filename_prefix}_result.jpg")
        cv2.imwrite(result_path, frame)
        self.get_logger().info(f"✅{camera_name}OCR结果已保存到:{result_path}")

        return ocr_results


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(vision_node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
