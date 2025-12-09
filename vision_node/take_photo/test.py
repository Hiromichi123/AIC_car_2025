# -*- coding: utf-8 -*-
import cv2

print("正在扫描可用的摄像头...")

for i in range(6):
    cap = cv2.VideoCapture(i)

    if not cap.isOpened():
        print(f"/dev/video{i} 无法打开")
        continue

    ret, frame = cap.read()
    if ret:
        print(f"✔ /dev/video{i} 可读取，分辨率 = {frame.shape}")
        cv2.imshow(f"Camera {i}", frame)
        cv2.waitKey(1000)  # 显示 1 秒
        cv2.destroyAllWindows()
    else:
        print(f"✘ /dev/video{i} 打开但无法读取帧")

    cap.release()
