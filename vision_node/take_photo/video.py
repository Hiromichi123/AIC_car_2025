# -*- coding: utf-8 -*-
import cv2
import time

def open_camera(dev_id, width=1920, height=1080, fps=30):
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头 /dev/video{dev_id}")

    return cap

def create_writer(filename, width, height, fps):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 通用、稳定
    return cv2.VideoWriter(filename, fourcc, fps, (width, height))

def main():
    # ===== 参数区 =====
    cam1_id = 4              # 第一个摄像头
    cam2_id = 6              # 第二个摄像头（按你实际修改）
    width, height = 1920, 1080
    fps = 30
    record_time = 10         # 录制时长（秒）

    # ===== 打开摄像头 =====
    cap1 = open_camera(cam1_id, width, height, fps)
    cap2 = open_camera(cam2_id, width, height, fps)

    # ===== 创建视频写入器 =====
    writer1 = create_writer("camera1.mp4", width, height, fps)
    writer2 = create_writer("camera2.mp4", width, height, fps)

    print("������ 开始录制...")
    start_time = time.time()

    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1 or not ret2:
            print("⚠ 读取帧失败，提前结束")
            break

        writer1.write(frame1)
        writer2.write(frame2)

        if time.time() - start_time > record_time:
            break

    print("✅ 录制完成，正在保存文件...")

    # ===== 释放资源 =====
    cap1.release()
    cap2.release()
    writer1.release()
    writer2.release()

    print("������ 视频已保存：camera1.mp4 / camera2.mp4")

if __name__ == "__main__":
    main()
