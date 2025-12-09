import cv2

cap = cv2.VideoCapture(0)

width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps    = cap.get(cv2.CAP_PROP_FPS)

print("当前摄像头分辨率：", int(width), "x", int(height))
print("当前摄像头 FPS：", fps)

cap.release()
