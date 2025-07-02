import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from msgs.srv import ObjectInformation, CheckAdultObj

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

        # ObjectInformation 서비스 등록
        self.create_service(ObjectInformation, '/obj_detect', self.handle_get_depth)
        self.get_logger().info("Service '/obj_detect' ready.")

        # CheckAdultObj 서비스 등록 (이제 server!)
        self.create_service(CheckAdultObj, '/check_adult_obj', self.handle_check_adult)
        self.get_logger().info("Service '/check_adult_obj' ready.")

        # Detection 결과
        self.latest_detections = None

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
    
    def draw_detections(self, image, detections):
        for det in detections:
            cx, cy, _, _, _ = det["box"]
            cx, cy = int(cx), int(cy)
            
            # 중심점 그리기 (작은 원)
            cv2.circle(image, (cx, cy), radius=5, color=(0, 255, 0), thickness=-1)
            
            # label 표시
            cv2.putText(image, det["label"], (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return image

    def handle_get_depth(self, request, response):
        if self.latest_detections:
            largest = self.latest_detections[0]
            box = largest["box"]
            label = largest["label"]
            cx, cy = int(box[0]), int(box[1])
            cz = self._get_depth(cx, cy)

            if cz is not None and cz > 0.0:
                radian = box[4]
                camera_coords = self._pixel_to_camera_coords(cx, cy, cz)

                fx = self.intrinsics['fx']
                min_px = min(box[2], box[3])
                min_size = (min_px * cz) / fx

                response.position = [float(x) for x in camera_coords] + [float(radian), float(min_size)]
                response.class_name = label

                self.get_logger().info(
                    f"Service response: position={response.position}, class_name={label}"
                )
            else:
                self.get_logger().warn("Invalid depth. Sending empty response.")
        else:
            self.get_logger().warn("No detections available yet. Sending empty response.")

        return response

    def handle_check_adult(self, request, response):
        """
        CheckAdultObj 서비스 콜백
        trigger=True 요청이 오면 현재 detection에 terra 클래스 있는지 확인 후 응답
        """
        if request.trigger:
            if self.latest_detections:
                terra_count = sum(1 for d in self.latest_detections if d["label"] == "terra")
                response.state_adult_event = terra_count > 0
                self.get_logger().info(f"CheckAdultObj request: terra_count={terra_count}, state_adult_event={response.state_adult_event}")
            else:
                response.state_adult_event = False
                self.get_logger().info("CheckAdultObj request: No detections, state_adult_event=False")
        else:
            response.state_adult_event = False
            self.get_logger().info("CheckAdultObj request: trigger=False, state_adult_event=False")

        return response
    
    def main_pipeline(self):
        self.get_logger().info("🚀 Starting main detection pipeline.")

        while rclpy.ok():
            # 이미지 업데이트
            rclpy.spin_once(self.img_node)

            detections = self.model.get_best_detection(self.img_node)
            if not detections:
                self.latest_detections = None
                self.get_logger().warn("No detections found.")
                continue

            self.latest_detections = sorted(
                detections, key=lambda d: d["box"][2] * d["box"][3], reverse=True
            )

            largest = self.latest_detections[0]
            label = largest["label"]
            box = largest["box"]
            self.get_logger().info(f"Detected: {label}, box={box}")

            # ---------- 이미지 시각화 ----------
            color_img = self.img_node.get_color_frame()
            if color_img is not None and color_img.any():
                vis_img = self.draw_detections(color_img.copy(), self.latest_detections)
                cv2.imshow("Real-time Detection", vis_img)
                key = cv2.waitKey(1)
                if key == 27:
                    self.get_logger().info("ESC pressed. Closing visualization.")
                    break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        node.main_pipeline()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
