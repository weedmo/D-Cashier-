import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool
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
        self.pause_state = True

        # 서비스 등록
        self.create_service(SetBool, '/pause_service', self.handle_pause)
        self.get_logger().info("Service '/pause_service' ready.")

        self.create_service(ObjectInformation, '/obj_detect', self.handle_get_depth)
        self.get_logger().info("Service '/obj_detect' ready.")

        # Detection 결과
        self.latest_detections = None
        self.sent_check_adult_once = False

        # CheckAdultObj 서비스 클라이언트
        self.check_adult_client = self.create_client(CheckAdultObj, '/check_adult_obj')
        while not self.check_adult_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/check_adult_obj service not available, waiting...')

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
        """
        detection 결과를 입력 이미지에 그려주는 함수
        """
        for det in detections:
            cx, cy, w, h, angle = det["box"]
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, det["label"], (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return image

    def send_check_adult_request(self):
        req = CheckAdultObj.Request()
        req.trigger = True

        future = self.check_adult_client.call_async(req)

        def callback(fut):
            try:
                result = fut.result()
                self.get_logger().info(f"✅ CheckAdultObj response: state_adult_event={result.state_adult_event}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(callback)

    def handle_pause(self, request, response):
        self.pause_state = request.data
        if self.pause_state:
            self.get_logger().info("✅ Resume signal received. Continuing detection.")
            response.success = True
            response.message = "Resumed"
        else:
            self.get_logger().info("⏸️ Paused. Waiting for resume command...")
            response.success = False
            response.message = "Paused"
        return response

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
    
    def main_pipeline(self):
        self.get_logger().info("🚀 Starting main detection pipeline.")

        while rclpy.ok():
            if not self.pause_state:
                self.get_logger().warn("⏸️ Detection paused. Waiting to resume...")
                rclpy.spin_once(self, timeout_sec=0.5)
                continue

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

            # terra 클래스 여부 확인
            terra_count = sum(1 for d in self.latest_detections if d["label"] == "terra")
            if terra_count > 0 and not self.sent_check_adult_once:
                self.send_check_adult_request()
                self.sent_check_adult_once = True

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