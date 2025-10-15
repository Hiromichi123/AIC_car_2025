import rclpy
from rclpy.node import Node

from ros2_tools.srv import YOLO
from ros2_tools.srv import OCR

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv_yolo = self.create_service(YOLO, 'yolo_trigger', self.yolo_callback)
        self.srv_ocr = self.create_service(OCR, 'ocr_trigger', self.ocr_callback)

    def yolo_callback(self, request, response):
        response.success = True
        response.message = "yolo callback!"
        return response

    def ocr_callback(self, request, response):
        response.success = True
        response.message = "ocr callback!"
        return response

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
