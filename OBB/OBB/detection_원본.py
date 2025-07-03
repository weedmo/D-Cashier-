import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any

from ament_index_python.packages import get_package_share_directory
from msgs.srv import ObjectInformation
from msgs.srv import AdultEvent

from OBB.realsense import ImgNode
from OBB.yolo import YoloModel

PACKAGE_NAME = 'OBB'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.get_logger().info("ObjectDetectionNode initialized.")

        self.create_service(
            ObjectInformation,
            '/obj_detect',
            self.handle_get_depth
        )
        self.get_logger().info("Service '/obj_detect' ready.")

        # OCR+Face detection service client 생성
        self.verify_cli = self.create_client(AdultEvent, '/adult_event')
        while not self.verify_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for OCR+Face detection service...')

    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        self.get_logger().info(f"Received request: {request}")

        admin_event = 2  # 기본값 (terra 탐지 안됨)

        detections = None
        while True:
            rclpy.spin_once(self.img_node)
            detections = self.model.get_best_detection(self.img_node)
            if detections:
                break
            self.get_logger().warn("No detection found. Retrying...")

        terra_count = sum(1 for d in detections if d["label"] == "terra")
        if terra_count > 0:
            self.get_logger().info(f"Detected {terra_count} terra object(s). Starting verification.")

            req = AdultEvent.Request()
            req.class_name = "start"
            future = self.verify_cli.call_async(req)

            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    try:
                        result = future.result()
                        if result.state_adult_event:
                            admin_event = 1  # OCR+Face 인증 성공
                            self.get_logger().info("✅ OCR+Face verification success. Proceeding to position estimation.")
                        else:
                            admin_event = 0  # OCR+Face 인증 실패
                            self.get_logger().warn("❌ OCR+Face verification failed.")
                        break
                    except Exception as e:
                        self.get_logger().error(f"Service call failed: {e}")
                        admin_event = 0
                        break
        else:
            self.get_logger().info("No terra detected. Skipping verification.")
            admin_event = 2

        # 이후 position 계산 진행
        coords, radian, num_classes, class_name, min_size = self._compute_position()

        response.position = [float(x) for x in coords] + [float(radian), float(min_size)]
        response.nums = num_classes
        response.class_name = class_name
        response.admin_event = admin_event

        self.get_logger().info(f"Response sent: position={response.position}, nums={num_classes}, class_name={class_name}, admin_event={admin_event}")
        return response

    def _compute_position(self):
        while True:
            rclpy.spin_once(self.img_node)

            detections = self.model.get_best_detection(self.img_node)
            if not detections:
                self.get_logger().warn("No detection found. Retrying...")
                continue

            def compute_area(det):
                box = det["box"]
                return box[2] * box[3]

            largest_det = max(detections, key=compute_area)
            box = largest_det["box"]
            label = largest_det["label"]

            self.get_logger().info(f"Largest Detection: label={label}, box={box}")

            cx, cy = int(box[0]), int(box[1])
            cz = self._get_depth(cx, cy)
            if cz is None or cz == 0.0:
                self.get_logger().warn("Depth out of range or zero. Retrying...")
                continue

            num_classes = len(set([d["label"] for d in detections]))
            radian = box[4]
            camera_coords = self._pixel_to_camera_coords(cx, cy, cz)
            # 1. min_size를 실세계 거리로 변환 (단위: mm)
            fx = self.intrinsics['fx']  # focal length in pixels
            min_px = min(box[2], box[3])  # width or height in pixels
            min_size = (min_px * cz) / fx  # 단위: mm

            return camera_coords, radian, num_classes, label, min_size

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