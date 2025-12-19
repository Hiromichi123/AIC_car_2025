from ultralytics import YOLO
from . utils import cut_square
import cv2
import os

# 获取模型文件的绝对路径
model_path = os.path.join("/home/jetson/ros2/AIC_car_2025/yolip/yolip/scripts/rubbish.pt")
model_cut = YOLO(model_path, verbose=False)

# 加载分类模型
cls_model_path = os.path.join("/home/jetson/ros2/AIC_car_2025/yolip/yolip/scripts/yolo_cls.pt")
model_cls = None
if os.path.exists(cls_model_path):
    model_cls = YOLO(cls_model_path, verbose=False)
    print(f"YOLO分类模型已加载:  {cls_model_path}")
else:
    print(f"警告：YOLO分类模型不存在: {cls_model_path}")


class YoloResultObj():
    confidence = None
    id = None
    name = None
    square_cut_img = None
    box = None

    def __init__(self, confidence=None, id=None, name=None, square_cut_img=None, box=None):
        if confidence != None:
            self.confidence = float(confidence)
        if id != None:
            self. id = int(id)
        if name: 
            self.name = str(name)
        self.square_cut_img = square_cut_img
        self. box = box
    
    def get_center_point(self):
        x1, y1, x2, y2 = map(int, self.box[: 4])
        center_x = int((x1 + x2)/2)
        center_y = int((y1 + y2)/2)
        return (center_x, center_y)


class YoloClsResult():
    """YOLO分类结果对象"""
    def __init__(self, full_name, category, item_name_only, confidence, center_x=0):
        self.full_name = full_name          # 完整名称:  "厨余垃圾-蘑菇-mushroom"
        self.item_name = item_name_only     # 仅物品名:  "蘑菇"
        self.category = category            # 大类:  "厨余垃圾"
        self.confidence = confidence
        self.center_x = center_x


def parse_class_name(class_name):
    """
    解析类别名称，提取大类和物品名
    格式: "大类-中文名-英文名" 例如: "厨余垃圾-蘑菇-mushroom"
    : return: (大类, 物品名, 完整名称)
    """
    if '-' in class_name:
        parts = class_name.split('-')
        if len(parts) >= 3:
            # 标准格式: "大类-中文名-英文名"
            category = parts[0]      # "厨余垃圾"
            item_name = parts[1]     # "蘑菇"
            return category, item_name, class_name
        elif len(parts) == 2:
            # 简化格式: "大类-中文名"
            category = parts[0]
            item_name = parts[1]
            return category, item_name, class_name
    
    # 降级：如果格式不对，尝试匹配关键词
    if '厨余' in class_name: 
        return '厨余垃圾', class_name, class_name
    elif '可回收' in class_name or '回收' in class_name: 
        return '可回收垃圾', class_name, class_name
    elif '有害' in class_name: 
        return '有害垃圾', class_name, class_name
    elif '其他' in class_name: 
        return '其他垃圾', class_name, class_name
    else:
        return '其他垃圾', class_name, class_name


def infer_cut(img, confidence_threshold:  float = 0.5, sort_left_to_right: bool = True) -> list[YoloResultObj]:
    """
    YOLO检测并裁切
    :param img: 输入图像
    :param confidence_threshold: 置信度阈值
    :param sort_left_to_right: 是否按从左到右排序
    :return: YoloResultObj列表（已排序）
    """
    results = model_cut(img, verbose=False)
    result_list = []

    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return result_list
    
    for i in range(len(boxes)):
        box = boxes. xyxy[i]. cpu().numpy().tolist()
        confidence = boxes.conf[i].item()
        cls_id = int(boxes.cls[i].item())
        name = model_cut.names[cls_id]

        if confidence >= confidence_threshold:
            r = YoloResultObj(
                confidence=confidence,
                id=cls_id,
                name=name,
                square_cut_img=cut_square(img, box),
                box=box,
            )
            result_list.append(r)
    
    if sort_left_to_right and result_list:
        result_list.sort(key=lambda obj: obj.get_center_point()[0])
    
    return result_list


def infer_classify(img, confidence_threshold:  float = 0.5, sort_left_to_right: bool = True) -> list[YoloClsResult]: 
    """使用YOLO分类模型直接识别垃圾类别"""
    if model_cls is None:
        raise RuntimeError("YOLO分类模型未加载")
    
    det_results = model_cut(img, verbose=False)
    result_list = []

    boxes = det_results[0].boxes
    if boxes is None or len(boxes) == 0:
        return result_list
    
    for i in range(len(boxes)):
        box = boxes. xyxy[i]. cpu().numpy().tolist()
        det_confidence = boxes.conf[i].item()
        
        if det_confidence < confidence_threshold: 
            continue
        
        x1, y1, x2, y2 = map(int, box[: 4])
        cropped_img = img[y1:y2, x1:x2]
        
        if cropped_img.size == 0:
            continue
        
        cls_results = model_cls(cropped_img, verbose=False)
        
        if cls_results and len(cls_results) > 0:
            probs = cls_results[0]. probs
            top1_idx = int(probs. top1)
            top1_conf = float(probs.top1conf)
            
            full_name = model_cls.names[top1_idx]
            
            # ========== 修改：分离提取大类和物品名 ==========
            category, item_name_only, full = parse_class_name(full_name)
            
            center_x = (x1 + x2) / 2
            
            result = YoloClsResult(
                full_name=full,                # "厨余垃圾-蘑菇-mushroom"
                category=category,             # "厨余垃圾"
                item_name_only=item_name_only, # "蘑菇"
                confidence=top1_conf,
                center_x=center_x
            )
            result_list.append(result)
    
    if sort_left_to_right and result_list:
        result_list.sort(key=lambda obj: obj.center_x)
    
    return result_list


def draw_yolo_box(img, yolo_result: YoloResultObj, color=(0, 0, 255), thickness=2):
    x1, y1, x2, y2 = map(int, yolo_result.box)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    confidence = yolo_result.confidence
    label = f"{confidence:.2f}"
    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)


def draw_all_yolo_boxes(img, yolo_results:  list[YoloResultObj], color=(0, 0, 255), thickness=2):
    if not yolo_results:
        return
    for i, yolo_result in enumerate(yolo_results):
        x1, y1, x2, y2 = map(int, yolo_result.box)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
        confidence = yolo_result.confidence
        name = yolo_result.name if hasattr(yolo_result, 'name') else f"obj_{i}"
        label = f"{name}:  {confidence:.2f}"
        cv2.putText(img, label, (x1, y1 - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), thickness)


if __name__ == "__main__": 
    img = cv2.imread("test. jpg", flags=1)
    if img is None: 
        raise FileNotFoundError("找不到图片")

    print("=== 测试检测模型 ===")
    results = infer_cut(img, sort_left_to_right=True)
    print(f"检测到 {len(results)} 个物体（从左到右）:")
    for i, result in enumerate(results):
        center_x, center_y = result.get_center_point()
        print(f"  物体{i+1}: {result.name}, 置信度:  {result.confidence:.2f}, 中心:  ({center_x}, {center_y})")

    if model_cls: 
        print("\n=== 测试分类模型 ===")
        cls_results = infer_classify(img, sort_left_to_right=True)
        print(f"识别到 {len(cls_results)} 个垃圾（从左到右）:")
        for i, result in enumerate(cls_results):
            print(f"  垃圾{i+1}:  {result.item_name}, 大类: {result.category}, 置信度: {result.confidence:.2f}")

    cv2.imshow("YOLOv8 Detection", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    