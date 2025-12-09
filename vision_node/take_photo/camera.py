import cv2

device_path = "/dev/video4"  # 你可以换成 /dev/video4 测试
cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)

# 可选：设置分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not cap.isOpened():
    print(f"无法打开摄像头：{device_path}")
    exit()

print(f"成功打开摄像头：{device_path}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取画面")
        break

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
