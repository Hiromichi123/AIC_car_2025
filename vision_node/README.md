# Vision Node

这是一个集成了YOLO目标检测和PaddleOCR文字识别的ROS2节点。

## 功能

1. **订阅话题**：
   - `/camera1/image_raw` - Camera1的原始图像（用于YOLO检测）
   - `/camera1/camera_info` - Camera1的相机信息
   - `/camera2/image_raw` - Camera2的原始图像（用于OCR识别）
   - `/camera2/camera_info` - Camera2的相机信息

2. **提供服务**：
   - `yolo_trigger` - 触发YOLO目标检测（使用camera1的图像）
   - `ocr_trigger` - 触发OCR文字识别（使用camera2的图像）

## 依赖安装

```bash
# 进入vision_node目录
cd /home/dev/ros_ws/rs_ws/src/AIC_car_2025/vision_node

# 安装Python依赖
pip install -r requirements.txt
```

## 编译

```bash
cd /home/dev/ros_ws/rs_ws
colcon build --packages-select vision_node
source install/setup.bash
```

## 运行

1. 启动vision_node：
```bash
ros2 run vision_node node
```

2. 调用YOLO检测服务：
```bash
ros2 service call /yolo_trigger ros2_tools/srv/YOLO
```

3. 调用OCR识别服务：
```bash
ros2 service call /ocr_trigger ros2_tools/srv/OCR
```

4. 或者使用测试脚本：
```bash
cd /home/dev/ros_ws/rs_ws/src/AIC_car_2025/vision_node
python3 test_vision_node.py
```

## 模型文件

- **YOLO模型**: `yolo/best.pt`
- **YOLO字体**: `yolo/NotoSansSC-VariableFont_wght.ttf`
- **YOLO结果保存**: `yolo/results/`
- **OCR结果保存**: `ocr/inference_results/`

## YOLO检测类别

- 类别1: 社区内人员（绿色框）
- 类别2: 非社区人员（红色框）

## 注意事项

1. 确保camera话题正在发布图像数据
2. YOLO检测使用camera1的图像
3. OCR识别使用camera2的图像
4. 检测结果会保存到相应的results目录
5. PaddleOCR使用CPU模式运行（如需GPU，修改node.py中的`use_gpu=True`）

## 故障排除

如果遇到以下问题：

### 1. 找不到PaddleOCR模块
确保已经安装了PaddleOCR：
```bash
pip install paddleocr
```

### 2. YOLO模型加载失败
检查模型文件路径是否正确：
```bash
ls /home/dev/ros_ws/rs_ws/src/AIC_car_2025/vision_node/yolo/best.pt
```

### 3. 没有接收到图像
检查camera话题是否在发布：
```bash
ros2 topic list | grep camera
ros2 topic echo /camera1/image_raw --no-arr
```

### 4. cv_bridge导入失败
安装cv_bridge：
```bash
sudo apt-get install ros-humble-cv-bridge python3-opencv
```
