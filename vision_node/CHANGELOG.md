# Vision Node 更新日志

## 2025-10-20 - 双摄像头YOLO+OCR支持

### 主要改进

#### 1. YOLO标签修正（参考test2_new.py）
- ✅ 修正了类别映射关系：
  - `0: "社区内人员"` (红色标注)
  - `1: "非社区人员"` (绿色标注)
- ✅ 之前的错误映射（1→社区内，2→非社区）已修正

#### 2. 双摄像头同时识别
- ✅ YOLO服务现在同时处理camera1和camera2的图像
- ✅ OCR服务现在同时处理camera1和camera2的图像
- ✅ 每个摄像头的结果独立标识（[Camera1]、[Camera2]）

#### 3. 输出优化
- ✅ 保存原始图像和结果图像
  - YOLO: `camera1_时间戳_raw.jpg` / `camera1_时间戳_result.jpg`
  - OCR: 类似命名规则
- ✅ 详细的日志输出，包含摄像头来源信息
- ✅ 中文标签绘制（使用PIL + TrueType字体）

#### 4. 代码结构优化
- ✅ 提取了 `_process_yolo_image()` 方法处理单张YOLO图像
- ✅ 提取了 `_process_ocr_image()` 方法处理单张OCR图像
- ✅ 减少代码重复，提高可维护性

### 使用示例

#### 运行节点
```bash
cd ~/ros_ws/rs_ws
source install/setup.bash
ros2 run vision_node node
```

#### 测试服务（使用测试脚本）
```bash
# 在另一个终端
source install/setup.bash
python3 src/AIC_car_2025/vision_node/test_vision_node.py
```

#### 手动调用服务
```bash
# YOLO检测（同时处理camera1和camera2）
ros2 service call /yolo_trigger ros2_tools/srv/YOLO

# OCR识别（同时处理camera1和camera2）
ros2 service call /ocr_trigger ros2_tools/srv/OCR
```

### 配置说明

在 `config.py` 中可以切换使用的模型：
- `CURRENT_MODEL = GREEN_MODEL`  # 使用绿色底牌模型
- `CURRENT_MODEL = BLUE_MODEL`   # 使用蓝色底牌模型
- `CURRENT_MODEL = DEFAULT_MODEL` # 使用PaddleOCR默认模型

### 结果文件位置

- **YOLO结果**: `/home/dev/ros_ws/rs_ws/src/AIC_car_2025/vision_node/yolo/results/`
  - `camera1_时间戳_raw.jpg` - Camera1原始图像
  - `camera1_时间戳_result.jpg` - Camera1检测结果
  - `camera2_时间戳_raw.jpg` - Camera2原始图像
  - `camera2_时间戳_result.jpg` - Camera2检测结果

- **OCR结果**: `/home/dev/ros_ws/rs_ws/src/AIC_car_2025/vision_node/ocr/inference_results/`
  - 类似的命名规则

### 技术细节

#### YOLO绘制（参考test2_new.py）
```python
# 1. 转换为PIL图像
img_pil = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
draw = ImageDraw.Draw(img_pil)

# 2. 加载中文字体
font = ImageFont.truetype(font_path, 28, encoding="utf-8")

# 3. 绘制框和标签
draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
draw.text((x1, y1 - 30), text, font=font, fill=color)

# 4. 转回OpenCV格式
frame = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
```

#### 双摄像头处理流程
```python
# 1. 检查两个相机是否有图像
# 2. 分别处理camera1和camera2（如果图像存在）
# 3. 汇总结果并返回
# 4. 每个摄像头的结果独立保存和标识
```

### 已知问题
- 如果某个摄像头未连接，会跳过该摄像头并给出警告
- 需要确保字体文件存在：`NotoSansSC-VariableFont_wght.ttf`

### 依赖要求
- ROS2 (Humble或更高)
- PaddleOCR
- Ultralytics YOLO
- OpenCV
- PIL (Pillow)
- cv_bridge

### 下一步计划
- [ ] 支持实时视频流显示
- [ ] 添加置信度阈值配置
- [ ] 支持ROI（感兴趣区域）配置
- [ ] 优化性能（多线程处理）
