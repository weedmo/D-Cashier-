#!/usr/bin/env python3
# 파일: detect_and_draw.py
import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory
from msgs.srv import ObjectInformation
from OBB.realsense import ImgNode
from OBB.yolo import YoloModel
from OBB.polygon_processor import PolygonProcessor

PKG = 'OBB'
PKG_PATH = get_package_share_directory(PKG)
BACK_PATH = os.path.join(PKG_PATH, 'resource', 'back.jpg')


class DetectAndDrawNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('detect_and_draw_node')

        # ------------- RealSense & 모델 -------------
        self.img_node = ImgNode()
        self.model = YoloModel() if model_name.lower() == 'yolo' else None
        self.intr = self._wait_for(lambda: self.img_node.get_camera_intrinsic(),
                                   "camera intrinsics")

        # ------------- 백그라운드 차분 -------------
        back_img = cv2.imread(BACK_PATH)
        self.polygon_processor = PolygonProcessor(back_img) if back_img is not None else None

        # ------------- 퍼블리셔 -------------
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image,
                                         '/annotated_image',
                                         QoSProfile(depth=1))

        # ------------- 타이머 -------------
        self.create_timer(0.05, self.timer_cb)   # 20 Hz

        self.get_logger().info("🟢 DetectAndDrawNode ready.")

    # --------------------------------------------------
    # Helper
    # --------------------------------------------------
    def _wait_for(self, getter, what):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"⏳ Waiting for {what} …")
            data = getter()
        self.get_logger().info(f"✅ {what} loaded.")
        return data

    def _depth(self, x, y):
        depth = self._wait_for(lambda: self.img_node.get_depth_frame(),
                               "depth frame")
        try:
            return float(depth[y, x])
        except IndexError:
            return None

    def _px2cam(self, x, y, z):
        fx, fy = self.intr['fx'], self.intr['fy']
        ppx, ppy = self.intr['ppx'], self.intr['ppy']
        return ( (x - ppx) * z / fx,
                 (y - ppy) * z / fy,
                 z )

    # --------------------------------------------------
    # 메인 루프
    # --------------------------------------------------
        # --------------------------------------------------
    # 메인 루프
    # --------------------------------------------------
    def timer_cb(self):
        # 최신 프레임 확보
        rclpy.spin_once(self.img_node)
        color = self.img_node.get_color_frame()
        if color is None or not color.any():
            return

        # ── 1) YOLO 탐지 ───────────────────────────────
        detections = self.model.get_best_detection(self.img_node) or []

        # ── 2) 백그라운드 차분 Fallback ────────────────
        if not detections and self.polygon_processor:
            diff = self.polygon_processor.process(color)
            if diff:
                detections.append(diff)

        if not detections:
            self._publish(color)
            return

        # ── 3) 모든 박스 오버레이 ──────────────────────
        for det in detections:
            box, label = det["box"], det["label"]
            cx, cy = int(box[0]), int(box[1])
            cz = self._depth(cx, cy)

            x1, y1 = int(cx - box[2] / 2), int(cy - box[3] / 2)
            x2, y2 = int(cx + box[2] / 2), int(cy + box[3] / 2)
            cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if cz and cz > 0.0:
                rad = box[4]
                x_cam, y_cam, z_cam = self._px2cam(cx, cy, cz)
                info = (f"{label}:({x_cam:.2f},{y_cam:.2f},{z_cam:.2f})m "
                        f"yaw={rad:.2f}")
                cv2.putText(color, info, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                cv2.circle(color, (cx, cy), 3, (0, 255, 0), -1)

        # ── 4) 퍼블리시 & 화면 표시 ────────────────────
        self._publish(color)


    # --------------------------------------------------
    # 이미지 퍼블리시 & 시각화
    # --------------------------------------------------
    def _publish(self, bgr):
        img_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        self.pub.publish(img_msg)
        cv2.imshow("Annotated", bgr)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC → 종료
            rclpy.shutdown()


# ======================================================
def main(args=None):
    rclpy.init(args=args)
    node = DetectAndDrawNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
