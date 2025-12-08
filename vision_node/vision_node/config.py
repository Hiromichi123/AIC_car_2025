"""Vision node 配置及路径管理。
默认情况下使用源码目录的绝对路径，同时允许在运行时通过 ROS 参数覆盖。"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, Optional

# 默认使用当前文件所在包的上级目录（vision_node 根目录）
_DEFAULT_SRC_DIR = Path(__file__).resolve().parents[1]

# 运行时可覆盖的全局变量（通过 configure_paths 更新）
VISION_NODE_SRC_DIR: str
OCR_DIR: str
YOLO_DIR: str
GREEN_MODEL: Dict[str, object]
BLUE_MODEL: Dict[str, object]
DEFAULT_MODEL: Dict[str, object]
CURRENT_MODEL: Dict[str, object]
OCR_SAVE_DIR: str
YOLO_MODEL_PATH: str
YOLO_FONT_PATH: str
YOLO_SAVE_DIR: str


def _build_ocr_model_config(base_dir: Path, variant: str) -> Dict[str, object]:
    """构建 OCR 模型配置，variant 取值例如 "green" 或 "blue"."""

    model_root = base_dir / "ocr" / "model" / variant
    return {
        "det_model_dir": str(model_root / "det" / "infer"),
        "rec_model_dir": str(model_root / "rec" / "infer"),
        "rec_char_dict_path": str(base_dir / "ocr" / "ppocr" / "utils" / "ppocr_keys_v1.txt"),
        "use_angle_cls": False,
        "lang": "ch",
        "use_gpu": False,
    }


def _apply_base_dir(base_dir: Path) -> None:
    """根据给定目录刷新所有路径配置."""
    global VISION_NODE_SRC_DIR, OCR_DIR, YOLO_DIR
    global GREEN_MODEL, BLUE_MODEL, DEFAULT_MODEL, CURRENT_MODEL
    global OCR_SAVE_DIR, YOLO_MODEL_PATH, YOLO_FONT_PATH, YOLO_SAVE_DIR

    base_dir = base_dir.resolve()
    VISION_NODE_SRC_DIR = str(base_dir)
    OCR_DIR = str(base_dir / "ocr")
    YOLO_DIR = str(base_dir / "yolo")

    # 本地训练模型配置
    GREEN_MODEL = _build_ocr_model_config(base_dir, "green")
    BLUE_MODEL = _build_ocr_model_config(base_dir, "blue")

    # 默认在线模型配置，保持可用
    DEFAULT_MODEL = {
        "use_angle_cls": True,
        "lang": "ch",
        "use_gpu": False,
        "show_log": False,
    }

    # 默认选用 BLUE 模型，可按需切换
    CURRENT_MODEL = BLUE_MODEL

    OCR_SAVE_DIR = str(base_dir / "ocr" / "inference_results")
    YOLO_MODEL_PATH = str(base_dir / "yolo" / "best2.pt")
    YOLO_FONT_PATH = str(base_dir / "yolo" / "NotoSansSC-VariableFont_wght.ttf")
    YOLO_SAVE_DIR = str(base_dir / "yolo" / "results")


def configure_paths(src_dir: Optional[str]) -> str:
    """使用给定源码目录刷新配置，返回解析后的绝对路径。
    如果传入 None 或空字符串，则继续使用当前配置。
    """
    if src_dir:
        candidate = Path(src_dir).expanduser()
        if not candidate.exists():
            raise ValueError(f"指定的 vision_node 源码目录不存在: {candidate}")
        _apply_base_dir(candidate)
    return VISION_NODE_SRC_DIR


# 初始化默认配置
_apply_base_dir(_DEFAULT_SRC_DIR)

# 自定义YOLO标签（参考test2_new.py）
YOLO_LABELS = {
    0: "社区内人员",
    1: "非社区人员"
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
