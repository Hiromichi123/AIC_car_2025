import cv2
import os
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO as YOLOModel  # type: ignore
from PIL import ImageFont, ImageDraw, Image as PILImage
from paddleocr import PaddleOCR

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

        # 自定义标签和颜色映射配置
        self.custom_color_map = {
            0: (255, 0, 0),     # 蓝框
            1: (0, 255, 0),     # 白框
            2: (0, 0, 255),     # 红框
            3: (0, 255, 255),   # 绿框
            4: (255, 255, 0),   # 黄框
            5: (255, 0, 255),   # 紫框
            6: (192, 192, 192), # 灰框
            7: (255, 165, 0),   # 橙框
        }
        
        # YOLO标签和颜色配置
        self.model_label_maps = {
            "traffic_light": {
                0: "无灯",
                1: "绿灯",
                2: "红灯",
                3: "黄灯",
            },
            "people": {
                0: "非社区人员",
                1: "社区人员",
            },
            "rubbish_bin": {
                0: "有害闭",
                1: "有害开",
                2: "厨余闭",
                3: "厨余开",
                4: "其他闭",
                5: "其他开",
                6: "可回收闭",
                7: "可回收开",
            },
            "e_bike": {
                0: "倒伏",
                1: "停放正确",
                2: "停放不正确",
            },
            "fire": {
                0: "火灾",
            },
            "default": {
                0: "target1",
                1: "target2",
                2: "target3",
                3: "target4",
                4: "target5",
                5: "target6",
                6: "target7",
            }
        }

        # 延迟初始化PaddleOCR - 在第一次使用时加载
        self.ocr_engine = None
        self._ocr_init_attempted = False
        
        self.ocr_save_dir = config.OCR_SAVE_DIR
        os.makedirs(self.ocr_save_dir, exist_ok=True)

        self.get_logger().info("VisionNode初始化完成")

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
                return self._handle_traffic_light_detection(request, response, model, model_name, camera_mode)
            
            # 垃圾桶检测 - 检测到后调用 yolip 服务
            if model_name == "rubbish_bin":
                return self._handle_rubbish_bin_detection(response, model, model_name, camera_mode)
            
            # 默认检测
            return self._handle_standard_detection(response, model, model_name, camera_mode)

        except Exception as e:
            self.get_logger().error(f"YOLO检测异常: {e}")
            response.success = False
            response.message = f"YOLO检测失败: {str(e)}"
            return response

    def _handle_traffic_light_detection(self, request, response, model, model_name, camera_mode):
        """交通灯检测 - 循环直到绿灯或超时"""
        max_wait = 20.0
        poll_interval = 0.5
        deadline = time.time() + max_wait

        while time.time() < deadline:
            ts = time.strftime("%Y%m%d-%H%M%S")
            
            # 获取当前相机图像
            current_images = self._get_camera_images(camera_mode)
            if not current_images: # 图像未就绪，继续等待
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

            # 添加调试日志
            self.get_logger().info(f"🔍 当前检测结果: {all_results}")

            # 检查是否检测到绿灯（支持中英文）
            green_hits = [msg for msg in all_results if ("绿灯" in msg or "green" in msg.lower())]
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

    def _handle_rubbish_bin_detection(self, response, model, model_name, camera_mode):
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
        
        # 手动等待结果，避免在回调中使用 spin_until_future_complete
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 10.0:
                break
            time.sleep(0.1)
        
        if future.done():
            try:
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
            except Exception as e:
                response.success = False
                response.message = f"垃圾分类服务调用异常: {str(e)}. 垃圾桶检测: {'; '.join(all_results)}"
                self.get_logger().error(f"yolip服务调用异常: {e}")
        else:
            response.success = False
            response.message = f"垃圾分类服务调用超时。垃圾桶检测: {'; '.join(all_results)}"
            self.get_logger().error("yolip服务调用超时")
        
        return response

    def _handle_standard_detection(self, response, model, model_name, camera_mode):
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
            self.get_logger().info(f"处理{cam_name}图像... 使用模型: {model_name}")
            det_results = self._process_yolo_image(
                frame.copy(),
                f"{cam_name}_{ts}_{model_name}",
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
            response.message = f"{', '.join(all_results)}"
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
        filename_prefix, # 保存文件名前缀
        camera_name,
        model, # YOLO模型实例
        model_name, # 模型名称,用于查找对应的标签配置
        suppress_empty_warn: bool = False, # 是否抑制空结果警告
        save_result: bool = True, # 是否保存结果图像
        return_frame: bool = False, # 是否返回处理后的图像
    ):
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

            # 获取自定义映射
            custom_map = self.model_label_maps.get(model_name)

            # 处理每个检测框
            for box in boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)

                # 使用自定义映射
                if custom_map and cls_id in custom_map:
                    label = custom_map[cls_id]
                    color = self.custom_color_map.get(cls_id, (255, 255, 255))
                else:
                    label = f"未知类别({cls_id})"
                    color = (255, 255, 255)

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 绘制矩形框
                draw.rectangle([x1, y1, x2, y2], outline=color, width=3)

                # 绘制中文标签
                text = f"{label} {conf:.2f}"
                draw.text((x1, y1 - 30), text, font=font, fill=color)

                # 记录结果
                result_str = f"{label}"
                detection_results.append(result_str)
        
        # 处理图像分类结果 (Probs) - 针对交通灯分类模型
        elif results and hasattr(results[0], 'probs') and results[0].probs is not None:
            has_detections = True
            probs = results[0].probs
            # 获取置信度最高的类别
            top1_index = int(probs.top1)
            conf = float(probs.top1conf)
            
            # 获取对应的label_map
            label_map = self.model_label_maps.get(model_name, self.model_label_maps["default"])
            
            # 获取中文标签
            display_label = label_map.get(top1_index, f"未知类别({top1_index})")
            
            result_str = f"{display_label} (置信度: {conf:.2f})"
            detection_results.append(result_str)
            
            # 在左上角绘制分类结果
            draw.text((30, 30), f"Type: {display_label} {conf:.2f}", font=font, fill=(0, 255, 0))

        # 转回OpenCV格式
        frame = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

        # 保存检测结果图像
        if save_result:
            result_path = os.path.join(
                self.yolo_save_dir, f"{filename_prefix}.jpg"
            )
            cv2.imwrite(result_path, frame)
            self.get_logger().info(f"✅ 结果已保存: {result_path}")
            
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

    def _ensure_ocr_engine(self):
        """延迟初始化OCR引擎，仅在首次调用时加载"""
        if self.ocr_engine is not None:
            return True
            
        if self._ocr_init_attempted:
            return False
            
        self._ocr_init_attempted = True
        
        try:
            self.get_logger().info("正在初始化PaddleOCR引擎...")
            
            # 尝试使用配置中的模型参数
            ocr_config = config.CURRENT_MODEL.copy()
            
            # 检查字典文件是否存在，如果不存在则移除该参数使用默认字典
            if "rec_char_dict_path" in ocr_config:
                dict_path = ocr_config["rec_char_dict_path"]
                if not os.path.exists(dict_path):
                    self.get_logger().warn(f"指定的字典文件不存在，将使用PaddleOCR默认字典")
                    del ocr_config["rec_char_dict_path"]
            
            self.get_logger().info(f"使用OCR配置: {ocr_config}")
            
            try:
                self.ocr_engine = PaddleOCR(**ocr_config)
                self.get_logger().info("PaddleOCR引擎加载成功 (本地模型)")
                return True
            except Exception as e:
                self.get_logger().warn(f"加载本地模型失败: {e}，尝试使用默认在线模型配置...")
                
                # 回退到默认配置
                default_config = config.DEFAULT_MODEL.copy()
                self.ocr_engine = PaddleOCR(**default_config)
                self.get_logger().info("PaddleOCR引擎加载成功 (默认模型)")
                return True
                
        except Exception as e:
            import traceback
            self.get_logger().error(f"PaddleOCR引擎加载失败: {e}")
            self.ocr_engine = None
            return False

    def ocr_callback(self, request, response):
        """OCR识别服务回调函数 - 同时处理camera1和camera2"""
        self.get_logger().info("开始OCR识别...")

        # 延迟初始化OCR引擎
        if not self._ensure_ocr_engine():
            response.success = False
            response.message = "OCR引擎初始化失败，请检查PaddleOCR和PaddlePaddle版本兼容性"
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
                response.message = f"{', '.join(all_ocr_results)}"
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
        self.get_logger().info(f"🔍 开始对{camera_name}进行OCR识别...")
        try:
            result = self.ocr_engine.ocr(frame, cls=True)
        except Exception as e:
            self.get_logger().error(f"OCR识别异常: {e}")
            result = None

        # 在图像上绘制检测结果
        vis_frame = frame.copy()
        
        # PaddleOCR返回格式: [[box, (text, confidence)], ...] 或 None
        if result and isinstance(result, list) and len(result) > 0 and result[0]:
            self.get_logger().info(f"✅ {camera_name}检测到 {len(result[0])} 个文本区域")
            for idx, line in enumerate(result[0]):
                try:
                    box = line[0]  # 四个角点坐标
                    text = line[1][0]
                    #confidence = line[1][1]　# 置信度
                    
                    result_str = f"{text}"
                    ocr_results.append(result_str)
                    self.get_logger().info(f"[{idx+1}] 识别到文本: {result_str}")
                    
                    # 绘制检测框
                    box_points = np.array(box, dtype=np.int32)
                    cv2.polylines(vis_frame, [box_points], True, (0, 255, 0), 2)
                    
                    # 绘制文本
                    cv2.putText(vis_frame, f"{text[:10]}", 
                               tuple(box_points[0]), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.8, (0, 0, 255), 2)
                except Exception as e:
                    self.get_logger().warn(f"处理第{idx+1}个检测结果时出错: {e}")
        else:
            self.get_logger().warn(f"⚠️{camera_name}未检测到任何文本区域")

        # 保存可视化结果图像
        result_path = os.path.join(self.ocr_save_dir, f"{filename_prefix}.jpg")
        cv2.imwrite(result_path, vis_frame)
        self.get_logger().info(f"✅ {camera_name}OCR结果已保存到: {result_path}")

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
