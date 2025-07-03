#!/usr/bin/env python3
import os
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from msgs.srv import CancelObject, ObjectInformation
from OBB.realsense import ImgNode
from OBB.yolo import YoloModel
from OBB.polygon_processor import PolygonProcessor

PACKAGE_NAME = 'OBB'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
BACK_PATH = os.path.join(PACKAGE_PATH, 'resource', 'back.jpg')


class CombinedDetectionNode(Node):
    def __init__(self, model_name: str = 'yolo'):
        super().__init__('combined_detection_node')

        # callback group for concurrent service handling
        cb_group = ReentrantCallbackGroup()

        # 1) ImgNode 스레드로 spin
        self.img_node = ImgNode()
        t = threading.Thread(target=rclpy.spin, args=(self.img_node,), daemon=True)
        t.start()

        # 2) 모델 로드
        self.model = self._load_model(model_name)

        # 3) 카메라 내부 파라미터 가져오기
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.get_logger().info("✅ Camera intrinsics loaded.")

        # 4) 폴리곤 프로세서 (백그라운드 차분용)
        background = cv2.imread(BACK_PATH)
        if background is None:
            self.get_logger().warn("❗ Background image not found.")
        self.polygon_processor = PolygonProcessor(background)

        # 5) 두 서비스 등록
        self.create_service(
            CancelObject,
            '/cancel_object',
            self.cancel_object_cb,
            callback_group=cb_group
        )
        self.get_logger().info("✅ Service '/cancel_object' ready.")

        self.create_service(
            ObjectInformation,
            '/obj_detect',
            self.handle_get_depth_cb,
            callback_group=cb_group
        )
        self.get_logger().info("✅ Service '/obj_detect' ready.")


    def _load_model(self, name: str):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")


    def _wait_for_valid_data(self, getter, description: str):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data


    def _get_depth(self, x: int, y: int):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return float(frame[y, x])
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None


    def _pixel_to_camera_coords(self, x: int, y: int, z: float):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


    def cancel_object_cb(self, request, response):
        target_class = request.class_name
        self.get_logger().info("🔎 [CancelObject] request received.")

        # 최신 이미지 가져오기
        rclpy.spin_once(self.img_node)
        detections = self.model.get_best_detection(self.img_node)

        # 요청한 클래스만 골라서
        det = next(d for d in detections if d["label"] == target_class)
        box = det["box"]
        cx, cy = int(box[0]), int(box[1])
        cz = self._get_depth(cx, cy)

        radian = box[4]
        camera_coords = self._pixel_to_camera_coords(cx, cy, cz)
        fx = self.intrinsics['fx']
        min_px = min(box[2], box[3])
        min_size = (min_px * cz) / fx

        response.position = [
            float(camera_coords[0]),
            float(camera_coords[1]),
            float(camera_coords[2]),
            float(radian),
            float(min_size)
        ]
        self.get_logger().info(f"✅ [CancelObject] response: {response.position}")
        return response


    def handle_get_depth_cb(self, request, response):
        self.get_logger().info("🔎 [ObjectInformation] request received.")

        # 최신 이미지 가져오기
        rclpy.spin_once(self.img_node)
        detections = self.model.get_best_detection(self.img_node)

        if not detections:
            # 백그라운드 차분 fallback
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

                response.position = [
                    float(camera_coords[0]),
                    float(camera_coords[1]),
                    float(camera_coords[2]),
                    float(radian),
                    float(min_size)
                ]
                response.adult_obj = False
                response.class_name = "None"
                return response
            response.adult_obj = False
            return response

        # 가장 큰 박스 선택
        largest = max(detections, key=lambda d: d["box"][2] * d["box"][3])
        box = largest["box"]
        label = largest["label"]
        cx, cy = int(box[0]), int(box[1])
        cz = self._get_depth(cx, cy)

        if cz and cz > 0.0:
            radian = box[4]
            camera_coords = self._pixel_to_camera_coords(cx, cy, cz)
            fx = self.intrinsics['fx']
            min_px = min(box[2], box[3])
            min_size = (min_px * cz) / fx

            response.position = [
                float(camera_coords[0]),
                float(camera_coords[1]),
                float(camera_coords[2]),
                float(radian),
                float(min_size)
            ]
            response.class_name = label
            # 예: 'terra' 라벨 개수 세어서 adult_obj 판단
            terra_count = sum(1 for d in detections if d["label"] == "terra")
            response.adult_obj = terra_count > 0

            self.get_logger().info(
                f"✅ [ObjectInformation] pos={response.position}, "
                f"class={response.class_name}, adult={response.adult_obj}"
            )
        else:
            self.get_logger().warn("Invalid depth. empty response.")
            response.adult_obj = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CombinedDetectionNode()

    # MultiThreadedExecutor 로 두 서비스를 병렬 처리
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
