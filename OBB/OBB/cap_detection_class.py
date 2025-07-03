import cv2
import numpy as np

class OBBYellowAnalyzer:
    def process_image(self, image_path):
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError("이미지를 불러올 수 없습니다.")

        analysis_results = []

        for poly in results:
            pts = poly.reshape(4, 2).astype(np.float32)

            # ✅ 순서 보정
            if not self.is_counter_clockwise(pts):
                pts = pts[::-1]

            rect, short_vec_norm, normal_vec, center = self.compute_geometry(pts)
            _, roi_image, yellow_mask = self.get_masks_and_roi(image, pts)

            ref_point = pts[0]
            near_mask, far_mask, near_count, far_count = self.classify_yellow_pixels(
                yellow_mask, center, normal_vec, ref_point
            )

            analysis_results.append({
                "center": center.tolist(),
                "near_yellow_count": near_count,
                "far_yellow_count": far_count,
                "polygon_pts": pts.tolist()
            })

        return analysis_results

    def compute_geometry(self, pts):
        """polygon으로부터 rect, 벡터, center 계산"""
        rect = cv2.minAreaRect(pts)
        (center_x, center_y), (w, h), angle = rect

        if w < h:
            angle_vec_angle = angle
        else:
            angle_vec_angle = angle + 90

        theta = np.deg2rad(angle_vec_angle)
        short_vec_norm = np.array([np.cos(theta), np.sin(theta)])
        normal_vec = np.array([-short_vec_norm[1], short_vec_norm[0]])
        center = np.array([center_x, center_y])

        return rect, short_vec_norm, normal_vec, center

    def get_masks_and_roi(self, image, pts):
        """polygon mask와 ROI, yellow mask 생성"""
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [pts.astype(np.int32)], 255)
        roi_image = cv2.bitwise_and(image, image, mask=mask)

        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        return mask, roi_image, yellow_mask

    def classify_yellow_pixels(self, yellow_mask, center, normal_vec, ref_point):
        """yellow_mask를 기준으로 near, far 분리 및 카운트"""
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

        return near_mask, far_mask, near_count, far_count
    
    def is_counter_clockwise(self, vertices):
        """polygon vertex 순서가 반시계 방향인지 확인"""
        n = len(vertices)
        area = 0.0
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            area += (x1 * y2 - x2 * y1)
        return area > 0

# 예시 사용법
if __name__ == "__main__":
    analyzer = OBBYellowAnalyzer("/home/jsbae/ros2_ws/src/OBB/resource/obj_detect_obb.pt")
    results = analyzer.process_image("OBB/resource/Bacchus_img_12.jpg")

    for idx, res in enumerate(results):
        print(f"Polygon #{idx + 1}")
        print(" Center:", res["center"])
        print(" Near yellow pixel count:", res["near_yellow_count"])
        print(" Far yellow pixel count:", res["far_yellow_count"])
        print(" Polygon points:", res["polygon_pts"])
        print("----------------------------")
