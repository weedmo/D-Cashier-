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
    """
    YOLO OBB(Oriented Bounding Box) 기반 객체 탐지를 수행하는 클래스.
    - 실시간 프레임 수집
    - YOLO 모델을 통한 다각형(Polygon) 기반 객체 검출
    - 검출된 객체의 중심 좌표, 크기(width/height), yaw(회전 각도)를 계산 후 집계
    """

    def __init__(self):
        """YOLO OBB 모델 초기화"""
        self.model = YOLO(YOLO_MODEL_PATH)

    def get_frames(self, img_node, duration=1.0):
        """
        ROS2 이미지 노드로부터 지정된 시간 동안 여러 frame을 수집한다.
        Args:
            img_node: 이미지 노드 객체
            duration: 프레임 수집 시간(초)

        Returns:
            frame 이미지 배열 리스트
        """
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
        ROS2 노드에서 frame들을 받고 YOLO OBB를 사용하여 객체를 검출하고,
        Polygon vertex로부터 중심 좌표, 크기, yaw 등을 종합하여 최종 결과를 반환한다.

        Returns:
            detections: [
                { "label": label_name, "box": [cx, cy, w, h, yaw] },
                ...
            ]
        """
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:
            return None

        # YOLO 모델 다중 프레임 추론
        results = self.model(frames, verbose=False, conf=0.6)

        # YOLO 클래스 이름
        class_names = results[0].names
        print("classes: ", class_names)

        # 검출된 polygon vertex들을 종합
        detections = self._aggregate_detections(results)
        print("detections: ", detections)

        # 클래스 이름 변환 및 결과 정리
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
        각 frame의 polygon vertex를 기반으로 center, width, height, yaw를 계산하고,
        비슷한 객체를 그룹화하여 평균값을 집계한다.

        Returns:
            list of dicts: [{ "box": [cx, cy, w, h, yaw], "label": class_id }, ...]
        """
        raw = []
        for res in results:
            for poly, score, label in zip(
                res.obb.xyxyxyxy.tolist(),  # polygon vertex
                res.obb.conf.tolist(),     # confidence
                res.obb.cls.tolist(),      # class id
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

            for g in group:
                poly = g["poly"]

                if poly is None or len(poly) != 4:
                    print(f"⚠️ Warning: Invalid polygon vertex count: {len(poly) if poly is not None else 'None'} -> {poly}")
                    continue

                vertices = poly

                # Polygon vertex 순서를 반시계 방향으로 보정
                if not self._is_counter_clockwise(vertices):
                    vertices = vertices[::-1]

                # 중심점 계산
                cx = np.mean([v[0] for v in vertices])
                cy = np.mean([v[1] for v in vertices])

                # 두 edge의 길이 계산 (긴 쪽이 width, 짧은 쪽이 height)
                edge1 = np.hypot(vertices[1][0] - vertices[0][0], vertices[1][1] - vertices[0][1])
                edge2 = np.hypot(vertices[2][0] - vertices[1][0], vertices[2][1] - vertices[1][1])
                w = max(edge1, edge2)
                h = min(edge1, edge2)

                # yaw 계산
                yaw = self._compute_yaw_from_vertices(vertices)

                # 개별 값 리스트에 저장
                centers_x.append(cx)
                centers_y.append(cy)
                widths.append(w)
                heights.append(h)
                yaws.append(yaw)

            # 평균 계산 (yaw는 sin, cos 방식)
            cx_mean = np.mean(centers_x)
            cy_mean = np.mean(centers_y)
            w_mean = np.mean(widths)
            h_mean = np.mean(heights)

            sin_sum = np.mean(np.sin(yaws))
            cos_sum = np.mean(np.cos(yaws))
            yaw_mean = np.arctan2(sin_sum, cos_sum)

            final.append({
                "box": [cx_mean, cy_mean, w_mean, h_mean, yaw_mean],
                "label": Counter([g["label"] for g in group]).most_common(1)[0][0],
            })

        return final

    def _iou_simple(self, poly1, poly2):
        """
        polygon 간 IoU(Intersection over Union)를 단순 계산한다.
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
        vertex들의 순서가 반시계 방향인지 확인 (signed area를 이용)
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
        polygon vertex로부터 가장 긴 edge를 찾아 yaw를 계산
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

        longest_edge = max(edges, key=lambda e: e['length'])
        theta = math.atan2(longest_edge['dy'], longest_edge['dx'])
        return theta
