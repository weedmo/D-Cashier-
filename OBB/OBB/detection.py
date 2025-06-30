import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any

from ament_index_python.packages import get_package_share_directory
from msgs.srv import ObjectInformation
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel

PACKAGE_NAME = 'msgs'
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
            'get_3d_position',
            self.handle_get_depth
        )
        self.get_logger().info("Service 'get_3d_position' ready.")

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords, radian, num_classes, class_name = self._compute_position()

        response.position = [float(x) for x in coords] + [float(radian)]
        response.nums = num_classes
        response.class_name = class_name

        self.get_logger().info(f"Response sent: position={response.position}, nums={num_classes}, class_name={class_name}")
        return response

    def _compute_position(self):
        """
        이미지를 처리해 bounding box 넓이가 가장 큰 객체의 카메라 좌표를 계산하고,
        동시에 전체 detection 클래스 개수와 radian 값을 반환.
        탐지가 될 때까지 무한히 반복.
        """
        while True:
            rclpy.spin_once(self.img_node)

            detections = self.model.get_best_detection(self.img_node)
            if not detections:
                self.get_logger().warn("No detection found. Retrying...")
                continue

            # 넓이 계산
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

            return camera_coords, radian, num_classes, label

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return float(frame[y, x])
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
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
