from ultralytics import YOLO
import cv2
import os
import sys

__dir__ = os.path.dirname(os.path.abspath(__file__))
print(__dir__)

# === 1. 加载模型 ===
model = YOLO(os.path.join(__dir__, "best.pt"))   # 替换为你的模型文件路径

# === 2. 输入图片路径 ===
image_path = "91AF11059C91D064E3A2934EADA2D1A4.jpg"         # 替换为要检测的图片路径
results = model(os.path.join(__dir__, image_path))

# === 3. 结果绘制与显示 ===
for result in results:
    # 使用plot()在图片上绘制检测框
    annotated_frame = result.plot()

    # 显示检测结果
    cv2.imshow("YOLOv8 Detection", annotated_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 保存结果图片（可选）
    cv2.imwrite("./result.jpg", annotated_frame)

# === 4. 输出检测信息（可选）===
for box in results[0].boxes:
    cls_id = int(box.cls[0])
    conf = float(box.conf[0])
    print(f"类别: {model.names[cls_id]}, 置信度: {conf:.2f}")
