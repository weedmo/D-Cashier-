import os
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from msgs.srv import ObjectInformation

from OBB.realsense import ImgNode
from OBB.yolo import YoloModel
from OBB.polygon_processor import PolygonProcessor  # PolygonProcessor 클래스 import

PACKAGE_NAME = 'OBB'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

BACK_NAME = "back.jpg"
BACK_PATH = os.path.join(PACKAGE_PATH, "resource", BACK_NAME)

class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.get_logger().info("✅ Camera intrinsics loaded.")

        # Background 이미지 불러오기
        background = cv2.imread(BACK_PATH)
        if background is None:
            self.get_logger().warn("❗ Background image not found.")
        self.polygon_processor = PolygonProcessor(background)

        # ObjectInformation 서비스 등록
        self.create_service(ObjectInformation, '/obj_detect', self.handle_get_depth)
        self.get_logger().info("✅ Service '/obj_detect' ready.")

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

    def handle_get_depth(self, request, response):
        self.get_logger().info("🔎 [ObjectInformation] Service request received.")

        # 이미지 업데이트
        rclpy.spin_once(self.img_node)

        detections = self.model.get_best_detection(self.img_node)
        if not detections:
            self.get_logger().warn("No detections found. Sending empty response.")
            result_diff = self.polygon_processor.process(self.img_node.get_color_frame())
            if result_diff:
                box = result_diff["box"]
                cx, cy = int(box[0]), int(box[1])
                cz = self._get_depth(cx, cy)
                radian = box[4]
                camera_coords = self._pixel_to_camera_coords(cx, cy, cz)
                fx = self.intrinsics['fx']
                min_px = min(box[2], box[3])
                min_size = (min_px * cz) / fx
                response.position = [float(x) for x in camera_coords] + [float(radian), float(min_size)]
                response.adult_obj = False
                response.class_name = "None Class"
                response.adult_obj = False
                return response
            
            response.adult_obj = False
            return response

        # 가장 큰 객체 선택
        detections = sorted(
            detections, key=lambda d: d["box"][2] * d["box"][3], reverse=True
        )
        largest = detections[0]
        box = largest["box"]
        label = largest["label"]
        cx, cy = int(box[0]), int(box[1])
        cz = self._get_depth(cx, cy)

        self.get_logger().info(f"Detected: {label}, coordi=({cx}, {cy}, {cz})")

        if cz is not None and cz > 0.0:
            radian = box[4]
            camera_coords = self._pixel_to_camera_coords(cx, cy, cz)
            fx = self.intrinsics['fx']
            min_px = min(box[2], box[3])
            min_size = (min_px * cz) / fx

            response.position = [float(x) for x in camera_coords] + [float(radian), float(min_size)]
            response.class_name = label

            # terra 객체 개수 세기
            terra_count = sum(1 for d in detections if d["label"] == "terra")
            response.adult_obj = terra_count > 0

            self.get_logger().info(f"✅ Response: position={response.position}, class_name={label}, adult_obj={response.adult_obj}")
        else:
            self.get_logger().warn("Invalid depth. Sending empty response.")
            response.adult_obj = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
