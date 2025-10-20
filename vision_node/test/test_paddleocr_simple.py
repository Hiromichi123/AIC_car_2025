#!/usr/bin/env python3
"""
测试精简版PaddleOCR
验证功能是否正常
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import cv2
import numpy as np
from paddleocr import PaddleOCR

def test_init():
    """测试初始化"""
    print("=" * 50)
    print("测试1: 初始化PaddleOCR")
    print("=" * 50)
    
    try:
        ocr = PaddleOCR(
            use_angle_cls=True,
            lang='ch',
            use_gpu=False,
            show_log=False
        )
        print("✅ 初始化成功")
        return ocr
    except Exception as e:
        print(f"❌ 初始化失败: {e}")
        return None


def test_ocr_with_array(ocr):
    """测试numpy数组输入"""
    print("\n" + "=" * 50)
    print("测试2: numpy数组输入")
    print("=" * 50)
    
    try:
        # 创建一个测试图像
        img = np.ones((100, 300, 3), dtype=np.uint8) * 255
        cv2.putText(img, "Test 123", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        
        result = ocr.ocr(img, det=True, rec=True, cls=True)
        print(f"✅ OCR识别成功")
        print(f"结果: {result}")
        return True
    except Exception as e:
        print(f"❌ OCR识别失败: {e}")
        return False


def test_ocr_with_file(ocr):
    """测试文件路径输入"""
    print("\n" + "=" * 50)
    print("测试3: 文件路径输入")
    print("=" * 50)
    
    # 查找测试图像
    test_images = [
        "/home/dev/ros_ws/vision/2018495E1810A148E7AC61FFC1EDE24D.jpg",
        "/home/dev/ros_ws/vision/91AF11059C91D064E3A2934EADA2D1A4.jpg",
        "/home/dev/ros_ws/vision/CC6BC72DF89653C0E9A5241C4FD39D14.jpg",
    ]
    
    for img_path in test_images:
        if os.path.exists(img_path):
            try:
                result = ocr.ocr(img_path, det=True, rec=True, cls=True)
                print(f"✅ 识别 {os.path.basename(img_path)} 成功")
                if result and result[0]:
                    print(f"   检测到 {len(result[0])} 个文本区域")
                    for line in result[0][:3]:  # 只显示前3个
                        text = line[1][0]
                        conf = line[1][1]
                        print(f"   - {text} (置信度: {conf:.2f})")
                return True
            except Exception as e:
                print(f"❌ 识别失败: {e}")
                continue
    
    print("⚠️ 未找到测试图像")
    return False


def test_det_only(ocr):
    """测试仅检测"""
    print("\n" + "=" * 50)
    print("测试4: 仅文本检测")
    print("=" * 50)
    
    try:
        img = np.ones((100, 300, 3), dtype=np.uint8) * 255
        cv2.putText(img, "Test Detection", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        
        result = ocr.ocr(img, det=True, rec=False, cls=False)
        print(f"✅ 仅检测成功")
        print(f"结果: {result}")
        return True
    except Exception as e:
        print(f"❌ 检测失败: {e}")
        return False


def test_rec_only(ocr):
    """测试仅识别"""
    print("\n" + "=" * 50)
    print("测试5: 仅文本识别")
    print("=" * 50)
    
    try:
        img = np.ones((32, 320, 3), dtype=np.uint8) * 255
        cv2.putText(img, "Recognition", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        result = ocr.ocr(img, det=False, rec=True, cls=False)
        print(f"✅ 仅识别成功")
        print(f"结果: {result}")
        return True
    except Exception as e:
        print(f"❌ 识别失败: {e}")
        return False


def main():
    print("\n")
    print("╔" + "=" * 50 + "╗")
    print("║" + " " * 10 + "PaddleOCR 精简版测试" + " " * 18 + "║")
    print("╚" + "=" * 50 + "╝")
    print()
    
    # 测试初始化
    ocr = test_init()
    if not ocr:
        print("\n❌ 初始化失败，无法继续测试")
        return
    
    # 运行测试
    results = []
    results.append(("numpy数组输入", test_ocr_with_array(ocr)))
    results.append(("文件路径输入", test_ocr_with_file(ocr)))
    results.append(("仅文本检测", test_det_only(ocr)))
    results.append(("仅文本识别", test_rec_only(ocr)))
    
    # 汇总结果
    print("\n" + "=" * 50)
    print("测试汇总")
    print("=" * 50)
    
    for name, success in results:
        status = "✅ 通过" if success else "❌ 失败"
        print(f"{name:20s} : {status}")
    
    total = len(results)
    passed = sum(1 for _, success in results if success)
    
    print("\n" + "-" * 50)
    print(f"总计: {passed}/{total} 测试通过")
    print("-" * 50)
    
    if passed == total:
        print("\n🎉 所有测试通过！精简版PaddleOCR工作正常！")
    else:
        print(f"\n⚠️ {total - passed} 个测试失败")


if __name__ == '__main__':
    main()
