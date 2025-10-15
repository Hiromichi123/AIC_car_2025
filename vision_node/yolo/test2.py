import cv2
import os
import time
import numpy as np
from ultralytics import YOLO
from PIL import ImageFont, ImageDraw, Image

# === 自定义标签 ===
custom_labels = {
    1: "社区内人员",
    2: "非社区人员"
}

# === 基本配置 ===
model_path = "/home/dev/ros_ws/vision/yolo/best.pt"
save_dir = "/home/dev/ros_ws/vision/yolo/results"
os.makedirs(save_dir, exist_ok=True)

# 字体路径（Windows 一般可用）
font_path = "NotoSansSC-VariableFont_wght.ttf"  # 微软雅黑字体

# 加载模型
model = YOLO(model_path)

# 打开摄像头
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ 无法打开摄像头")
    exit()

# 拍一帧
ret, frame = cap.read()
if not ret:
    print("⚠️ 无法读取摄像头画面")
    cap.release()
    exit()

timestamp = time.strftime("%Y%m%d-%H%M%S")
raw_path = os.path.join(save_dir, f"capture_{timestamp}.jpg")
cv2.imwrite(raw_path, frame)
print(f"📸 已保存原始图像: {raw_path}")

# YOLO 检测
results = model(frame)
boxes = results[0].boxes

# 将 frame 转为 Pillow 图像（方便绘制中文）
img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
draw = ImageDraw.Draw(img_pil)
font = ImageFont.truetype(font_path, 28, encoding="utf-8")

for box in boxes:
    cls_id = int(box.cls)
    conf = float(box.conf)
    label = custom_labels.get(cls_id, f"未知类别({cls_id})")

    x1, y1, x2, y2 = map(int, box.xyxy[0])
    color = (0, 255, 0) if cls_id == 1 else (255, 0, 0)

    # 绘制框
    draw.rectangle([x1, y1, x2, y2], outline=color, width=3)

    # 绘制中文标签
    text = f"{label} {conf:.2f}"
    draw.text((x1, y1 - 30), text, font=font, fill=color)

    print(f"Detected: {label} ({conf:.2f})")

# 转回 OpenCV 图像
frame = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

# 保存结果
result_path = os.path.join(save_dir, f"result_{timestamp}.jpg")
cv2.imwrite(result_path, frame)
print(f"✅ 检测结果已保存到: {result_path}")

# 显示结果直到按 q 退出
cap.release()
print("🟢 按 'q' 键退出窗口。")
while True:
    cv2.imshow("YOLOv8 检测结果", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
print("程序结束。")
