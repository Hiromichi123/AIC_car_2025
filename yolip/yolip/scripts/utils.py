from copy import copy
import math

# 将yolo框转为正方形图像
def cut_square(img, box):
    x1, y1, x2, y2 = map(int, box[:4])
    img_h, img_w = img.shape[:2]
    side = max([x2-x1, y2-y1]) + 10
    # 限制最大边长
    side = min(side, img_w, img_h)

    center_x = int((x1 + x2)/2)
    center_y = int((y1 + y2)/2) 
    sx1 = center_x - side // 2
    sy1 = center_y - side // 2
    sx2 = sx1 + side
    sy2 = sy1 + side
    
    # 边界修正
    if sx1 < 0:
        sx2 -= sx1
        sx1 = 0
    if sy1 < 0:
        sy2 -= sy1
        sy1 = 0
    if sx2 > img_w:
        sx1 -= (sx2 - img_w)
        sx2 = img_w
    if sy2 > img_h:
        sy1 -= (sy2 - img_h)
        sy2 = img_h
    sx1, sx2 = max(sx1, 0), min(sx2, img_w)
    sy1, sy2 = max(sy1, 0), min(sy2, img_h)
    square_cut_img = img[sy1:sy2, sx1:sx2]
    return square_cut_img

# 找到距离相机中心最近的矩形框
def find_nearest(results, image_shape)-> 'YoloResultObj':
    result = None
    min_distance = float('inf')
    h, w = image_shape[:2]
    center_x, center_y = w / 2, h / 2
    for r in results:
        cx, cy = r.get_center_point()
        distance = math.hypot(cx - center_x, cy - center_y)
        if distance < min_distance:
            min_distance = distance
            result = r

    return result
