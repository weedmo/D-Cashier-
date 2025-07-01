########## YoloModel ##########
import os
import json
import time
from collections import Counter
import cv2

import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np


PACKAGE_NAME = "OBB"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

YOLO_MODEL_FILENAME = "obj_detect_obb.pt"

YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)

class YoloModel:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)

    def get_frames(self, img_node, duration=1.0):
        """get frames while target_time"""
        end_time = time.time() + duration
        frames = {}

        while time.time() < end_time:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            stamp = img_node.get_color_frame_stamp()
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)

        if not frames:
            print("No frames captured in %.2f seconds", duration)

        print("%d frames captured", len(frames))
        return list(frames.values())

    def get_best_detection(self, img_node):
        """
        target 없이, 모든 라벨에 대해 모든 detection 결과 반환.

        return: [ { "label": label_name, "box": [...], "score": float }, ... ]
        """
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:  # Check if frames are empty
            return None

        results = self.model(frames, verbose=False)

        class_names = results[0].names
        
        print("classes: ")
        print(results[0].names)
        detections = self._aggregate_detections(results)
        print("detections: ", detections)

        # 클래스 이름으로 변환
        all_detections = []
        for det in detections:
            label_name = class_names.get(det["label"], f"id:{det['label']}")
            all_detections.append({
                "label": label_name,
                "box": det["box"],
            })

        return all_detections

    def _aggregate_detections(self, results, confidence_threshold=0.5, iou_threshold=0.5):
        """
        Fuse raw OBB detection boxes across frames using IoU-based grouping
        and robust angle averaging.

        results: list of YOLO OBB result objects
        """
        raw = []
        for res in results:
            for box, score, label in zip(
                res.obb.xywhr.tolist(),  # [cx, cy, w, h, rad]
                res.obb.conf.tolist(),
                res.obb.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"box": box, "score": score, "label": int(label)})

        final = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou_obb(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            boxes = np.array([g["box"] for g in group])
            labels = [g["label"] for g in group]

            # 평균 위치와 크기
            cx_mean = boxes[:, 0].mean()
            cy_mean = boxes[:, 1].mean()
            w_mean = boxes[:, 2].mean()
            h_mean = boxes[:, 3].mean()

            # yaw (rad) 평균 (sin/cos 방식)
            angles = boxes[:, 4]
            sin_sum = np.sin(angles).mean()
            cos_sum = np.cos(angles).mean()
            avg_angle = np.arctan2(sin_sum, cos_sum)

            final.append(
                {
                    "box": [cx_mean, cy_mean, w_mean, h_mean, avg_angle],
                    "label": Counter(labels).most_common(1)[0][0],
                }
            )

        return final

    def _iou_obb(self, box1, box2):
        # box: [cx, cy, w, h, rad]
        rect1 = ((box1[0], box1[1]), (box1[2], box1[3]), np.degrees(box1[4]))
        rect2 = ((box2[0], box2[1]), (box2[2], box2[3]), np.degrees(box2[4]))

        retval, intersecting_region = cv2.rotatedRectangleIntersection(rect1, rect2)

        if retval == 0 or intersecting_region is None:
            return 0.0

        inter_area = cv2.contourArea(intersecting_region)

        area1 = box1[2] * box1[3]
        area2 = box2[2] * box2[3]
        union_area = area1 + area2 - inter_area

        if union_area == 0:
            return 0.0
        return inter_area / union_area