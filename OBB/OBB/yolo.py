########## YoloModel ##########
import os
import time
from collections import Counter
import cv2

import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np
import math

PACKAGE_NAME = "OBB"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

YOLO_MODEL_FILENAME = "obj_detect_obb.pt"
YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)

class YoloModel:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)

    def get_frames(self, img_node, duration=1.0):
        """지정 시간 동안 frame들을 받아오는 함수"""
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
            print("No frames captured in %.2f seconds" % duration)

        print("%d frames captured" % len(frames))
        return list(frames.values())

    def get_best_detection(self, img_node):
        """
        frame들을 통해 얻은 모든 detection 결과 반환
        return: [ { "label": label_name, "box": [...] }, ... ]
        """
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:
            return None

        # YOLO 모델 inference
        results = self.model(frames, verbose=False, conf=0.6)

        # 클래스 이름 가져오기
        class_names = results[0].names
        print("classes: ", class_names)

        # 그룹화 및 개별 frame vertex 계산
        detections = self._aggregate_detections(results)
        print("detections: ", detections)

        # 클래스 이름 변환
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
        frame별 polygon vertex를 통해 center, w, h, yaw를 개별 계산 후, 각 param 평균
        """
        raw = []
        for res in results:
            for poly, score, label in zip(
                res.obb.xyxyxyxy.tolist(),  # [x1,y1,...,x4,y4]
                res.obb.conf.tolist(),
                res.obb.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"poly": poly, "score": score, "label": int(label)})

        final = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou_simple(det["poly"], other["poly"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            centers_x, centers_y, widths, heights, yaws = [], [], [], [], []

            # frame별 vertex 기반 center, w, h, yaw 계산
            for g in group:
                poly = g["poly"]
                # poly가 이미 [[x1,y1], [x2,y2], [x3,y3], [x4,y4]] 형태
                if poly is None or len(poly) != 4:
                    print(f"⚠️ Warning: Invalid polygon vertex count: {len(poly) if poly is not None else 'None'} -> {poly}")
                    continue

                vertices = poly  # ✅ 바로 사용

                # 순서 보정
                if not self._is_counter_clockwise(vertices):
                    vertices = vertices[::-1]

                cx = np.mean([v[0] for v in vertices])
                cy = np.mean([v[1] for v in vertices])

                edge1 = np.hypot(vertices[1][0] - vertices[0][0], vertices[1][1] - vertices[0][1])
                edge2 = np.hypot(vertices[2][0] - vertices[1][0], vertices[2][1] - vertices[1][1])
                w = max(edge1, edge2)
                h = min(edge1, edge2)

                yaw = self._compute_yaw_from_vertices(vertices)

                centers_x.append(cx)
                centers_y.append(cy)
                widths.append(w)
                heights.append(h)
                yaws.append(yaw)

            # param 평균
            cx_mean = np.mean(centers_x)
            cy_mean = np.mean(centers_y)
            w_mean = np.mean(widths)
            h_mean = np.mean(heights)

            # yaw 평균 (sin, cos 방식)
            sin_sum = np.mean(np.sin(yaws))
            cos_sum = np.mean(np.cos(yaws))
            yaw_mean = np.arctan2(sin_sum, cos_sum)

            final.append(
                {
                    "box": [cx_mean, cy_mean, w_mean, h_mean, yaw_mean],
                    "label": Counter([g["label"] for g in group]).most_common(1)[0][0],
                }
            )

        return final

    def _iou_simple(self, poly1, poly2):
        """
        간단한 polygon IoU 계산 (cv2 contours 기반)
        """
        contour1 = np.array(poly1, dtype=np.float32).reshape(-1, 1, 2)
        contour2 = np.array(poly2, dtype=np.float32).reshape(-1, 1, 2)

        inter = cv2.intersectConvexConvex(contour1, contour2)[0]
        area1 = cv2.contourArea(contour1)
        area2 = cv2.contourArea(contour2)
        union = area1 + area2 - inter

        if union == 0:
            return 0.0
        return inter / union

    def _is_counter_clockwise(self, vertices):
        """
        polygon vertex 순서가 반시계 방향인지 확인
        """
        n = len(vertices)
        area = 0.0
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            area += (x1 * y2 - x2 * y1)
        return area > 0

    def _compute_yaw_from_vertices(self, vertices):
        """
        vertex 기반으로 yaw (theta) 계산
        """
        edges = []
        n = len(vertices)
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            edges.append({'dx': dx, 'dy': dy, 'length': length})

        # 가장 긴 edge 기준으로 yaw 계산
        longest_edge = max(edges, key=lambda e: e['length'])
        theta = math.atan2(longest_edge['dy'], longest_edge['dx'])
        return theta