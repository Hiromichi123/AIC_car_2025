# Copyright (c) 2020 PaddlePaddle Authors. All Rights Reserved.
# 精简版 - 仅保留OCR核心功能

import os
import sys
import importlib
import importlib.util
import logging
import cv2
import numpy as np
from pathlib import Path

# 确保当前目录在 sys.path 中
__dir__ = os.path.dirname(os.path.abspath(__file__))
if __dir__ not in sys.path:
    sys.path.insert(0, __dir__)

# 确保 tools 和 ppocr 目录可访问
tools_dir = os.path.join(__dir__, "tools")
if tools_dir not in sys.path:
    sys.path.insert(0, tools_dir)

def _import_file(module_name, file_path, make_importable=False):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None:
        raise ImportError(f"Cannot create module spec for {module_name} from {file_path}")
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise ImportError(f"No loader found for {module_name} from {file_path}")
    spec.loader.exec_module(module)
    if make_importable:
        sys.modules[module_name] = module
    return module

# 导入必要模块 - 使用绝对路径导入
try:
    # 首先尝试作为包导入
    from ppocr.utils.logging import get_logger
    from ppocr.utils.utility import alpha_to_color
    from ppocr.utils.network import maybe_download, confirm_model_dir_url
    from tools.infer import predict_system
    from tools.infer.utility import str2bool, check_gpu, init_args
except ImportError:
    # 如果失败，使用文件路径导入
    tools_init_path = os.path.join(__dir__, "tools", "__init__.py")
    tools = _import_file("tools", tools_init_path, make_importable=True)
    
    ppocr_init_path = os.path.join(__dir__, "ppocr", "__init__.py")
    ppocr = _import_file("ppocr", ppocr_init_path, make_importable=True)
    
    # 导入具体模块
    from ppocr.utils.logging import get_logger
    from ppocr.utils.utility import alpha_to_color
    from ppocr.utils.network import maybe_download, confirm_model_dir_url
    from tools.infer import predict_system
    from tools.infer.utility import str2bool, check_gpu, init_args

logger = get_logger()

__all__ = ["PaddleOCR"]

SUPPORT_DET_MODEL = ["DB"]
SUPPORT_REC_MODEL = ["CRNN", "SVTR_LCNet"]
BASE_DIR = os.environ.get("PADDLE_OCR_BASE_DIR", os.path.expanduser("~/.paddleocr/"))

DEFAULT_OCR_MODEL_VERSION = "PP-OCRv4"
SUPPORT_OCR_MODEL_VERSION = ["PP-OCRv4", "PP-OCRv3"]

# 精简的模型URL配置 - 仅保留中文和英文
MODEL_URLS = {
    "OCR": {
        "PP-OCRv4": {
            "det": {
                "ch": {"url": "https://paddleocr.bj.bcebos.com/PP-OCRv4/chinese/ch_PP-OCRv4_det_infer.tar"},
                "en": {"url": "https://paddleocr.bj.bcebos.com/PP-OCRv3/english/en_PP-OCRv3_det_infer.tar"},
            },
            "rec": {
                "ch": {
                    "url": "https://paddleocr.bj.bcebos.com/PP-OCRv4/chinese/ch_PP-OCRv4_rec_infer.tar",
                    "dict_path": "./ppocr/utils/ppocr_keys_v1.txt",
                },
                "en": {
                    "url": "https://paddleocr.bj.bcebos.com/PP-OCRv4/english/en_PP-OCRv4_rec_infer.tar",
                    "dict_path": "./ppocr/utils/en_dict.txt",
                },
            },
            "cls": {
                "ch": {"url": "https://paddleocr.bj.bcebos.com/dygraph_v2.0/ch/ch_ppocr_mobile_v2.0_cls_infer.tar"}
            },
        },
        "PP-OCRv3": {
            "det": {
                "ch": {"url": "https://paddleocr.bj.bcebos.com/PP-OCRv3/chinese/ch_PP-OCRv3_det_infer.tar"},
                "en": {"url": "https://paddleocr.bj.bcebos.com/PP-OCRv3/english/en_PP-OCRv3_det_infer.tar"},
            },
            "rec": {
                "ch": {
                    "url": "https://paddleocr.bj.bcebos.com/PP-OCRv3/chinese/ch_PP-OCRv3_rec_infer.tar",
                    "dict_path": "./ppocr/utils/ppocr_keys_v1.txt",
                },
                "en": {
                    "url": "https://paddleocr.bj.bcebos.com/PP-OCRv3/english/en_PP-OCRv3_rec_infer.tar",
                    "dict_path": "./ppocr/utils/en_dict.txt",
                },
            },
            "cls": {
                "ch": {"url": "https://paddleocr.bj.bcebos.com/dygraph_v2.0/ch/ch_ppocr_mobile_v2.0_cls_infer.tar"}
            },
        },
    },
}


def parse_args(mMain=False):
    """简化的参数解析"""
    import argparse
    
    parser = init_args()
    parser.add_help = mMain
    parser.add_argument("--lang", type=str, default="ch")
    parser.add_argument("--det", type=str2bool, default=True)
    parser.add_argument("--rec", type=str2bool, default=True)
    parser.add_argument("--ocr_version", type=str, choices=SUPPORT_OCR_MODEL_VERSION, default="PP-OCRv4")
    
    if mMain:
        return parser.parse_args()
    else:
        inference_args_dict = {}
        for action in parser._actions:
            inference_args_dict[action.dest] = action.default
        return argparse.Namespace(**inference_args_dict)


def parse_lang(lang):
    """简化的语言解析 - 仅支持中英文"""
    if lang in ["ch", "chinese"]:
        det_lang = "ch"
        rec_lang = "ch"
    elif lang in ["en", "english"]:
        det_lang = "en"
        rec_lang = "en"
    else:
        logger.warning(f"Language {lang} not explicitly supported, using 'ch' as default")
        det_lang = "ch"
        rec_lang = "ch"
    return rec_lang, det_lang


def get_model_config(version, model_type, lang):
    """获取模型配置"""
    model_urls = MODEL_URLS["OCR"]
    
    if version not in model_urls:
        version = DEFAULT_OCR_MODEL_VERSION
        logger.warning(f"Using default version: {version}")
    
    if model_type not in model_urls[version]:
        logger.error(f"{model_type} not supported")
        sys.exit(-1)
    
    if lang not in model_urls[version][model_type]:
        logger.error(f"Language {lang} not supported for {model_type}")
        sys.exit(-1)
    
    return model_urls[version][model_type][lang]


class PaddleOCR(predict_system.TextSystem):
    """简化的PaddleOCR类 - 仅保留核心OCR功能"""
    
    def __init__(self, **kwargs):
        """
        初始化PaddleOCR
        
        常用参数:
            lang (str): 语言，'ch' 或 'en'，默认 'ch'
            use_angle_cls (bool): 是否使用方向分类器，默认 True
            use_gpu (bool): 是否使用GPU，默认 False
            show_log (bool): 是否显示日志，默认 False
        """
        params = parse_args(mMain=False)
        params.__dict__.update(**kwargs)
        
        assert params.ocr_version in SUPPORT_OCR_MODEL_VERSION, \
            f"ocr_version must in {SUPPORT_OCR_MODEL_VERSION}, but got {params.ocr_version}"
        
        params.use_gpu = check_gpu(params.use_gpu)
        
        if not params.show_log:
            logger.setLevel(logging.INFO)
        
        self.use_angle_cls = params.use_angle_cls
        lang, det_lang = parse_lang(params.lang)
        
        # 初始化模型目录
        det_model_config = get_model_config(params.ocr_version, "det", det_lang)
        params.det_model_dir, det_url = confirm_model_dir_url(
            params.det_model_dir,
            os.path.join(BASE_DIR, "whl", "det", det_lang),
            det_model_config["url"],
        )
        
        rec_model_config = get_model_config(params.ocr_version, "rec", lang)
        params.rec_model_dir, rec_url = confirm_model_dir_url(
            params.rec_model_dir,
            os.path.join(BASE_DIR, "whl", "rec", lang),
            rec_model_config["url"],
        )
        
        cls_model_config = get_model_config(params.ocr_version, "cls", "ch")
        params.cls_model_dir, cls_url = confirm_model_dir_url(
            params.cls_model_dir,
            os.path.join(BASE_DIR, "whl", "cls"),
            cls_model_config["url"],
        )
        
        # 设置图像形状
        if params.ocr_version in ["PP-OCRv3", "PP-OCRv4"]:
            params.rec_image_shape = "3, 48, 320"
        else:
            params.rec_image_shape = "3, 32, 320"
        
        if kwargs.get("rec_image_shape") is not None:
            params.rec_image_shape = kwargs.get("rec_image_shape")
        
        # 下载模型
        if not params.use_onnx:
            maybe_download(params.det_model_dir, det_url)
            maybe_download(params.rec_model_dir, rec_url)
            maybe_download(params.cls_model_dir, cls_url)
        
        # 验证算法支持
        if params.det_algorithm not in SUPPORT_DET_MODEL:
            logger.error(f"det_algorithm must in {SUPPORT_DET_MODEL}")
            sys.exit(0)
        if params.rec_algorithm not in SUPPORT_REC_MODEL:
            logger.error(f"rec_algorithm must in {SUPPORT_REC_MODEL}")
            sys.exit(0)
        
        # 设置字典路径
        if params.rec_char_dict_path is None:
            params.rec_char_dict_path = str(Path(__file__).parent / rec_model_config["dict_path"])
        
        logger.debug(params)
        
        # 初始化检测和识别模型
        super().__init__(params)
        self.page_num = params.page_num
    
    def ocr(self, img, det=True, rec=True, cls=True):
        """
        执行OCR识别
        
        参数:
            img: 输入图像，可以是numpy数组或图像路径
            det (bool): 是否进行文本检测，默认True
            rec (bool): 是否进行文本识别，默认True
            cls (bool): 是否使用角度分类器，默认True
        
        返回:
            list: OCR结果列表
                如果 det=True, rec=True: 返回 [[[box], (text, confidence)], ...]
                如果 det=True, rec=False: 返回 [[box], ...]
                如果 det=False, rec=True: 返回 [(text, confidence), ...]
        """
        assert isinstance(img, (np.ndarray, str)), "img must be numpy array or image path"
        
        if cls and not self.use_angle_cls:
            logger.warning("Angle classifier not initialized, cls will be ignored")
        
        # 处理图像
        if isinstance(img, str):
            if not os.path.exists(img):
                logger.error(f"Image file not found: {img}")
                return None
            img = cv2.imread(img)
            if img is None:
                logger.error(f"Failed to read image")
                return None
        
        # 确保图像是3通道BGR格式
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif len(img.shape) == 3 and img.shape[2] == 4:
            img = alpha_to_color(img, (255, 255, 255))
        
        imgs = [img]
        
        if det and rec:
            # 检测 + 识别
            ocr_res = []
            for image in imgs:
                dt_boxes, rec_res, _ = self.__call__(image, cls, {})
                # 避免 None 或 空结果导致类型/运行错误
                if dt_boxes is None or rec_res is None:
                    ocr_res.append(None)
                    continue
                if len(dt_boxes) == 0 or len(rec_res) == 0:
                    ocr_res.append(None)
                    continue
                tmp_res = [[box.tolist(), res] for box, res in zip(dt_boxes, rec_res)]
                ocr_res.append(tmp_res)
            return ocr_res
        
        elif det and not rec:
            # 仅检测
            ocr_res = []
            for image in imgs:
                dt_boxes, elapse = self.text_detector(image)
                if dt_boxes is None or len(dt_boxes) == 0:
                    ocr_res.append(None)
                    continue
                tmp_res = [box.tolist() for box in dt_boxes]
                ocr_res.append(tmp_res)
            return ocr_res
        
        else:
            # 仅识别
            ocr_res = []
            for image in imgs:
                if not isinstance(image, list):
                    image = [image]
                if self.use_angle_cls and cls:
                    image, _, _ = self.text_classifier(image)
                rec_res, _ = self.text_recognizer(image)
                ocr_res.append(rec_res)
            return ocr_res
