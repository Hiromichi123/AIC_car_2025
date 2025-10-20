#!/usr/bin/env python3
"""
测试green模型是否能正确加载

这个脚本独立于ROS2节点，直接测试PaddleOCR是否能加载green模型
"""

import os
import sys

# 添加vision_node到Python路径
vision_node_path = "/home/dev/ros_ws/rs_ws/src/AIC_car_2025/vision_node"
sys.path.insert(0, vision_node_path)

from ocr.paddleocr import PaddleOCR

def test_green_model():
    """测试加载green模型"""
    
    print("=" * 60)
    print("测试加载green模型")
    print("=" * 60)
    
    # 模型路径（相对于vision_node根目录）
    det_model_dir = os.path.join(vision_node_path, "ocr/model/green/det/infer")
    rec_model_dir = os.path.join(vision_node_path, "ocr/model/green/rec/infer")
    rec_char_dict_path = os.path.join(vision_node_path, "ocr/ppocr/utils/ppocr_keys_v1.txt")
    
    print(f"\n检测模型路径: {det_model_dir}")
    print(f"识别模型路径: {rec_model_dir}")
    print(f"字符字典路径: {rec_char_dict_path}")
    
    # 检查模型文件是否存在
    print("\n检查模型文件:")
    det_model_file = os.path.join(det_model_dir, "inference.pdmodel")
    det_params_file = os.path.join(det_model_dir, "inference.pdiparams")
    rec_model_file = os.path.join(rec_model_dir, "inference.pdmodel")
    rec_params_file = os.path.join(rec_model_dir, "inference.pdiparams")
    
    files_to_check = [
        ("检测模型文件", det_model_file),
        ("检测参数文件", det_params_file),
        ("识别模型文件", rec_model_file),
        ("识别参数文件", rec_params_file),
    ]
    
    all_exist = True
    for name, path in files_to_check:
        exists = os.path.exists(path)
        status = "✓" if exists else "✗"
        print(f"  {status} {name}: {exists}")
        if not exists:
            all_exist = False
    
    if not all_exist:
        print("\n❌ 模型文件不完整，无法继续测试")
        return False
    
    print("\n✓ 所有模型文件存在")
    
    # 尝试加载模型
    print("\n正在加载PaddleOCR模型...")
    try:
        ocr = PaddleOCR(
            det_model_dir=det_model_dir,
            rec_model_dir=rec_model_dir,
            rec_char_dict_path=rec_char_dict_path,
            use_angle_cls=False,
            lang='ch',
            use_gpu=False,
            show_log=True  # 显示详细日志
        )
        print("\n✓ PaddleOCR模型加载成功!")
        
        # 测试一下OCR功能（使用已有的测试图片）
        test_image = "/home/dev/ros_ws/vision/2018495E1810A148E7AC61FFC1EDE24D.jpg"
        if os.path.exists(test_image):
            print(f"\n测试OCR识别: {test_image}")
            result = ocr.ocr(test_image, cls=False)
            
            if result and result[0]:
                print(f"\n识别结果:")
                for idx, line in enumerate(result[0]):
                    box, (text, confidence) = line
                    print(f"  {idx+1}. 文字: {text}")
                    print(f"     置信度: {confidence:.4f}")
                    print(f"     坐标: {box}")
                print(f"\n✓ OCR识别成功，共识别出 {len(result[0])} 行文字")
            else:
                print("\n⚠ 未识别出文字（可能图片中没有文字）")
        else:
            print(f"\n⚠ 测试图片不存在: {test_image}")
            print("  跳过OCR识别测试")
        
        return True
        
    except Exception as e:
        print(f"\n❌ PaddleOCR模型加载失败:")
        print(f"   错误类型: {type(e).__name__}")
        print(f"   错误信息: {e}")
        import traceback
        print("\n详细错误信息:")
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("\nPaddleOCR Green模型加载测试")
    print("=" * 60)
    
    success = test_green_model()
    
    print("\n" + "=" * 60)
    if success:
        print("✓ 测试通过 - green模型可以正常使用")
    else:
        print("✗ 测试失败 - 请检查上述错误信息")
    print("=" * 60)
    
    sys.exit(0 if success else 1)
