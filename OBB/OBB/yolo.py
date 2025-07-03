import os
import time
from collections import Counter
import cv2
import numpy as np
import math
from ultralytics import YOLO
import rclpy
from ament_index_python.packages import get_package_share_directory

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
        """여러 frame으로부터 YOLO detection 결과 집계"""
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:
            return None

        results = self.model(frames, verbose=False, conf=0.6)
        class_names = results[0].names
        detections = self._aggregate_detections(results)

        all_detections = []
        for det in detections:
            label_name = class_names.get(det["label"], f"id:{det['label']}")
            all_detections.append({
                "label": label_name,
                "box": det["box"],
                "avg_poly": det["avg_poly"]
            })
        return all_detections

    def _aggregate_detections(self, results, confidence_threshold=0.5, iou_threshold=0.5):
        """여러 frame polygon vertex를 평균 집계하여 최종 box + poly 계산"""
        raw = []
        for res in results:
            for poly, score, label in zip(
                res.obb.xyxyxyxy.tolist(),
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
            group_vertices = []

            for g in group:
                poly = g["poly"]
                if poly is None or len(poly) != 4:
                    continue

                vertices = poly
                if not self.is_counter_clockwise(vertices):
                    vertices = vertices[::-1]

                group_vertices.append(vertices)

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

            if not group_vertices:
                continue

            cx_mean = np.mean(centers_x)
            cy_mean = np.mean(centers_y)
            w_mean = np.mean(widths)
            h_mean = np.mean(heights)
            sin_sum = np.mean(np.sin(yaws))
            cos_sum = np.mean(np.cos(yaws))
            yaw_mean = np.arctan2(sin_sum, cos_sum)

            avg_poly = np.mean(np.array(group_vertices), axis=0).tolist()

            final.append({
                "box": [cx_mean, cy_mean, w_mean, h_mean, yaw_mean],
                "label": Counter([g["label"] for g in group]).most_common(1)[0][0],
                "avg_poly": avg_poly
            })

        return final

    def analyze_yellow_in_polygon(self, img_node, box, polygon):
        """polygon 내부 yellow pixel 분석"""
        rclpy.spin_once(img_node)
        image = img_node.get_color_frame()
        if image is None:
            raise ValueError("이미지를 img_node로부터 받아올 수 없습니다.")

        normal_vec, center = self.compute_geometry(box)
        yellow_mask = self.get_masks_and_roi(image, np.array(polygon))

        ref_point = polygon[0]
        near_count, far_count = self.classify_yellow_pixels(
            yellow_mask, center, normal_vec, ref_point
        )

        return {
            "near_yellow_count": near_count,
            "far_yellow_count": far_count,
        }

    def compute_geometry(self, box):
        center_x, center_y, w, h, angle = box

        if w < h:
            angle_vec_angle = angle
        else:
            angle_vec_angle = angle + 90

        theta = np.deg2rad(angle_vec_angle)
        short_vec_norm = np.array([np.cos(theta), np.sin(theta)])
        normal_vec = np.array([-short_vec_norm[1], short_vec_norm[0]])
        center = np.array([center_x, center_y])

        return normal_vec, center

    def get_masks_and_roi(self, image, pts):
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [pts.astype(np.int32)], 255)
        roi_image = cv2.bitwise_and(image, image, mask=mask)

        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        return yellow_mask

    def classify_yellow_pixels(self, yellow_mask, center, normal_vec, ref_point):
        ref_vec = ref_point - center
        dot_ref = np.dot(ref_vec, normal_vec)
        ys, xs = np.where(yellow_mask > 0)
        near_mask = np.zeros_like(yellow_mask)
        far_mask = np.zeros_like(yellow_mask)
        near_count = 0
        far_count = 0

        for x_pixel, y_pixel in zip(xs, ys):
            pix_vec = np.array([x_pixel, y_pixel]) - center
            dot = np.dot(pix_vec, normal_vec)

            if (dot * dot_ref) > 0:
                near_count += 1
                near_mask[y_pixel, x_pixel] = 255
            else:
                far_count += 1
                far_mask[y_pixel, x_pixel] = 255

        return near_count, far_count

    def is_counter_clockwise(self, vertices):
        n = len(vertices)
        area = 0.0
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            area += (x1 * y2 - x2 * y1)
        return area > 0

    def _compute_yaw_from_vertices(self, vertices):
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

    def _iou_simple(self, poly1, poly2):
        contour1 = np.array(poly1, dtype=np.float32).reshape(-1, 1, 2)
        contour2 = np.array(poly2, dtype=np.float32).reshape(-1, 1, 2)

        inter = cv2.intersectConvexConvex(contour1, contour2)[0]
        area1 = cv2.contourArea(contour1)
        area2 = cv2.contourArea(contour2)
        union = area1 + area2 - inter

        return inter / union if union != 0 else 0.0