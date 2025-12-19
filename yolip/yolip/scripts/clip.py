import torch 
from PIL import Image
import cv2
import yaml
import logging
import os
import sys

CN_CLIP_PATH = "/home/jetson/ros2/AIC_car_2025/yolip/yolip"
if CN_CLIP_PATH not in sys.path:
    sys.path.insert(0, CN_CLIP_PATH)

import cn_clip.clip as clip
from cn_clip.clip import load_from_name

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 获取文件的绝对路径
yaml_path = os.path.join("/home/jetson/ros2/AIC_car_2025/yolip/yolip/scripts/list_cn.yaml")

# 读取垃圾分类配置
with open(yaml_path, 'r', encoding='utf-8') as f:
    category_dict = yaml.load(f.read(), Loader=yaml.FullLoader)

# 构建提示词列表和类别映射
prompt_words = []  # 所有小类标签的扁平列表
item_to_category = {}  # 小类 -> 大类的映射

for category, items in category_dict.items():
    for item in items:
        prompt_words.append(item)
        item_to_category[item] = category

logger.info(f"加载垃圾分类标签: {len(prompt_words)} 个小类，{len(category_dict)} 个大类")
logger.info(f"大类: {list(category_dict.keys())}")

device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = load_from_name("ViT-H-14", device=device, download_root="/home/jetson/ros2/AIC_car_2025/yolip/yolip/")
model.eval()
logger.info("CLIP模型加载成功，使用: {}".format(device))

class ClipResultObj():
    """CLIP识别结果对象"""
    confidence = None  # 置信度
    item_name = None   # 小类名称（具体物品）
    category = None    # 大类名称（垃圾分类）

    def __init__(self, confidence=None, item_name=None, category=None):
        if confidence is not None:
            self.confidence = float(confidence)
        if item_name:
            self.item_name = str(item_name)
        if category:
            self.category = str(category)
    
    # 保持向后兼容
    @property
    def name(self):
        return self.item_name

# 推理
def infer(img) -> list[ClipResultObj]:
    """对图像进行CLIP推理，返回所有可能的分类结果"""
    img = preprocess(Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))).unsqueeze(0).to(device)
    text = clip.tokenize(prompt_words).to(device)

    with torch.no_grad():
        image_features = model.encode_image(img)
        text_features = model.encode_text(text)
        # 对特征进行归一化
        image_features /= image_features.norm(dim=-1, keepdim=True) 
        text_features /= text_features.norm(dim=-1, keepdim=True)    

        logits_per_image, logits_per_text = model.get_similarity(img, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        # 构建结果列表
        probs = list(probs[0])
        results = []
        for i in range(len(probs)):
            item_name = prompt_words[i]
            category = item_to_category[item_name]
            conf = probs[i]
            
            if conf > 0.3:  # 只记录置信度较高的结果
                logger.info(f"CLIP识别:[{category}]{item_name}({conf:.3f})")
            
            results.append(
                ClipResultObj(
                    item_name=item_name,
                    category=category,
                    confidence=conf
                )
            )
        return results

def get_best_result(results: list[ClipResultObj], min_confidence=0.3) -> ClipResultObj:
    """从结果列表中获取置信度最高的结果"""
    valid_results = [r for r in results if r.confidence > min_confidence]
    if not valid_results:
        return None
    return max(valid_results, key=lambda r: r.confidence)

# 绘制CLIP识别结果
def draw_clip_result(img, yolo_result, clip_result: ClipResultObj):
    """在图像上绘制CLIP识别结果（大类和小类）"""
    x1, y1, x2, y2 = map(int, yolo_result.box)
    
    # 绘制大类（垃圾分类）
    category_text = f"{clip_result.category}"
    cv2.putText(img, category_text, (x2 + 10, y1 + 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 绘制小类（具体物品）和置信度
    item_text = f"{clip_result.item_name}: {clip_result.confidence:.2f}"
    cv2.putText(img, item_text, (x2 + 10, y1 + 45), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
