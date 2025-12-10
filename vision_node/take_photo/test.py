import cv2
import os
import subprocess

def list_video_devices():
    devices = []
    for dev in os.listdir('/dev'):
        if dev.startswith('video'):
            devices.append(f"/dev/{dev}")
    return sorted(devices)


def get_v4l2_info(device):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", device, "--all"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return result.stdout
    except Exception as e:
        return f"读取失败: {e}"


def test_capture(device):
    print(f"\n============================")
    print(f"测试摄像头: {device}")
    print("============================")

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"⚠ 无法打开 {device}")
        return

    ret, frame = cap.read()
    if not ret:
        print(f"⚠ 无法从 {device} 读取画面")
        cap.release()
        return

    print(f"原始 frame shape：{frame.shape}")

    # 判断是否是 UYVY（单通道）
    if len(frame.shape) == 2:
        print("检测到单通道灰度图 → 尝试 UYVY 转 BGR")
        try:
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_UYVY)
            print("转换后 frame shape：", frame.shape)
        except:
            print("❌ UYVY 转换失败")

    print("按 q 退出显示画面...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 自动处理灰度转彩色
        if len(frame.shape) == 2:
            try:
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_UYVY)
            except:
                pass

        cv2.imshow(device, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("正在检测所有摄像头...\n")

    devices = list_video_devices()
    if not devices:
        print("没有发现任何 /dev/video* 设备！")
        exit()

    print("发现设备：")
    for d in devices:
        print("  →", d)

    for d in devices:
        print("\n******** V4L2 信息 ********")
        print(get_v4l2_info(d))
        test_capture(d)
