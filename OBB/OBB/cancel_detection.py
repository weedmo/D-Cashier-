import numpy as np

import rclpy
from rclpy.node import Node

from msgs.srv import CancelObject

from OBB.realsense import ImgNode
from OBB.yolo import YoloModel

class CancelDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('cancel_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.get_logger().info("✅ Camera intrinsics loaded.")

        # ObjectInformation 서비스 등록
        self.create_service(CancelObject, '/cancel_object', self.cancel_object)
        self.get_logger().info("✅ Service '/cancel_object' ready.")

        self.last_detections = []

    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return float(frame[y, x])
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )
    
    def cancel_object(self, request, response):
        target_class = request.class_name
        self.get_logger().info("🔎 [ObjectInformation] Service request received.")

        # 이미지 업데이트
        rclpy.spin_once(self.img_node)

        detections = self.model.get_best_detection(self.img_node)
        self.get_logger().info(f"🚨 [CancelObject] Request for class_name: {target_class}")

        # 항상 존재한다고 가정
        det = next(d for d in detections if d["label"] == target_class)
        box = det["box"]
        cx, cy = int(box[0]), int(box[1])
        cz = self._get_depth(cx, cy)

        radian = box[4]
        camera_coords = self._pixel_to_camera_coords(cx, cy, cz)
        fx = self.intrinsics['fx']
        min_px = min(box[2], box[3])
        min_size = (min_px * cz) / fx

        response.position = [float(x) for x in camera_coords] + [float(radian), float(min_size)]

        self.get_logger().info(f"✅ [CancelObject] Response: position={response.position}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CancelDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()