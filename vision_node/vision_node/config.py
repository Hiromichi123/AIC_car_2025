"""
Vision Node OCR 配置文件
用于配置OCR模型路径和参数
"""

import os

# 使用绝对路径指向源码目录
# 无论从哪里运行（源码或install目录），都使用源码目录的资源
VISION_NODE_SRC_DIR = "/home/dev/ros_ws/rs_ws/src/AIC_car_2025/vision_node"

# 源码中的资源目录
OCR_DIR = os.path.join(VISION_NODE_SRC_DIR, "ocr")
YOLO_DIR = os.path.join(VISION_NODE_SRC_DIR, "yolo")

# Green模型配置（自定义训练的本地模型）
GREEN_MODEL = {
    "det_model_dir": os.path.join(OCR_DIR, "model", "green", "det", "infer"),
    "rec_model_dir": os.path.join(OCR_DIR, "model", "green", "rec", "infer"),
    "rec_char_dict_path": os.path.join(OCR_DIR, "ppocr", "utils", "ppocr_keys_v1.txt"),
    "use_angle_cls": False,
    "lang": "ch",
    "use_gpu": False,
}

# 默认PaddleOCR模型配置（在线下载）
DEFAULT_MODEL = {
    "use_angle_cls": True,
    "lang": "ch",
    "use_gpu": False,
    "show_log": False,
    # 不指定det_model_dir和rec_model_dir，使用默认下载的模型
}

# 当前使用的模型配置
# 可以在这里切换：GREEN_MODEL 或 DEFAULT_MODEL
CURRENT_MODEL = GREEN_MODEL

# OCR结果保存目录（保存到源码目录）
OCR_SAVE_DIR = os.path.join(OCR_DIR, "inference_results")

# YOLO配置（使用源码目录的绝对路径）
YOLO_MODEL_PATH = os.path.join(YOLO_DIR, "best.pt")
YOLO_FONT_PATH = os.path.join(YOLO_DIR, "NotoSansSC-VariableFont_wght.ttf")
YOLO_SAVE_DIR = os.path.join(YOLO_DIR, "results")

# 自定义YOLO标签
YOLO_LABELS = {
    1: "社区内人员",
    2: "非社区人员"
}

# 调试信息：打印路径
if __name__ == "__main__":
    print("=" * 60)
    print("Vision Node 配置路径")
    print("=" * 60)
    print(f"源码根目录: {VISION_NODE_SRC_DIR}")
    print(f"\nOCR目录: {OCR_DIR}")
    print(f"YOLO目录: {YOLO_DIR}")
    print(f"\n检测模型: {GREEN_MODEL['det_model_dir']}")
    print(f"识别模型: {GREEN_MODEL['rec_model_dir']}")
    print(f"字符字典: {GREEN_MODEL['rec_char_dict_path']}")
    print(f"\nYOLO模型: {YOLO_MODEL_PATH}")
    print(f"YOLO字体: {YOLO_FONT_PATH}")
    print("=" * 60)
    
    # 检查文件是否存在
    print("\n文件存在性检查:")
    files_to_check = [
        ("OCR检测模型", os.path.join(GREEN_MODEL['det_model_dir'], "inference.pdmodel")),
        ("OCR识别模型", os.path.join(GREEN_MODEL['rec_model_dir'], "inference.pdmodel")),
        ("字符字典", GREEN_MODEL['rec_char_dict_path']),
        ("YOLO模型", YOLO_MODEL_PATH),
        ("YOLO字体", YOLO_FONT_PATH),
    ]
    
    for name, path in files_to_check:
        exists = "✓" if os.path.exists(path) else "✗"
        print(f"{exists} {name:>8}: {path}")
