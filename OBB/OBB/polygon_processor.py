import cv2
import numpy as np

class PolygonProcessor:
    def __init__(self, background_image):
        if background_image is None:
            raise ValueError("Background image is None")
        self.background = background_image
        self.roi_vertices = [(48, 3), (526, 6), (529, 416), (50, 413)]  # ROI polygon vertices

    def compute_yaw_from_long_edge(self, vertices):
        edges = [
            (vertices[1][0] - vertices[0][0], vertices[1][1] - vertices[0][1]),
            (vertices[2][0] - vertices[1][0], vertices[2][1] - vertices[1][1]),
            (vertices[3][0] - vertices[2][0], vertices[3][1] - vertices[2][1]),
            (vertices[0][0] - vertices[3][0], vertices[0][1] - vertices[3][1])
        ]
        lengths = [np.hypot(dx, dy) for dx, dy in edges]
        max_idx = np.argmax(lengths)
        dx, dy = edges[max_idx]

        yaw = np.arctan2(dy, dx)

        if yaw > np.pi / 2:
            yaw -= np.pi
        elif yaw < -np.pi / 2:
            yaw += np.pi

        return yaw

    def is_counter_clockwise(self, vertices):
        sum = 0.0
        n = len(vertices)
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            sum += (x2 - x1) * (y2 + y1)
        return sum < 0

    def process_and_visualize(self, current_image, show=True, window_name="Polygon Result"):
        result = self.process(current_image)
        if result and show:
            cv2.imshow(window_name, result["poly_img"])
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return result

    def process(self, current_image):
        if current_image is None:
            raise ValueError("Current image is None")

        if self.background.shape != current_image.shape:
            current_image = cv2.resize(current_image, (self.background.shape[1], self.background.shape[0]))

        # ROI polygon 마스크 생성
        mask = np.zeros(self.background.shape[:2], dtype=np.uint8)
        if self.roi_vertices is not None:
            roi_pts = np.array(self.roi_vertices, dtype=np.int32)
            cv2.fillPoly(mask, [roi_pts], 255)

        diff = cv2.absdiff(current_image, self.background)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)

        # ROI 마스크 적용
        if self.roi_vertices is not None:
            thresh = cv2.bitwise_and(thresh, mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        poly_img = current_image.copy()

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1500:
                continue

            rect = cv2.minAreaRect(cnt)
            (cx, cy), (w, h), angle = rect
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            vertices = [[int(x), int(y)] for x, y in box]

            if not self.is_counter_clockwise(vertices):
                vertices = vertices[::-1]

            yaw = self.compute_yaw_from_long_edge(vertices)

            cv2.drawContours(poly_img, [box], 0, (0, 255, 0), 2)
            cv2.circle(poly_img, (int(cx), int(cy)), 5, (0, 0, 255), -1)

            for i, (px, py) in enumerate(vertices):
                cv2.circle(poly_img, (px, py), 5, (255, 0, 255), -1)
                text = f"({px},{py})"
                cv2.putText(poly_img, text, (px + 5, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

            yaw_text = f"Yaw: {np.degrees(yaw):.1f} deg"
            cv2.putText(poly_img, yaw_text, (int(cx) + 10, int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

            return {
                "box": (int(cx), int(cy), float(w), float(h), yaw),
                "poly_img": poly_img,
                "vertices": vertices,
                "yaw": yaw
            }

        return None

if __name__ == "__main__":
    background = cv2.imread("images5/back.jpg")
    current = cv2.imread("images5/test_img_1.jpg")

    # 클릭한 좌표로 ROI polygon 지정
    roi_vertices = [(48, 3), (526, 6), (529, 416), (50, 413)]

    processor = PolygonProcessor(background)

    result = processor.process_and_visualize(current)

    if result:
        print(f"box: {result['box']}")
        print(f"Yaw (deg): {np.degrees(result['yaw']):.2f}")
        print(f"Vertices: {result['vertices']}")
    else:
        print("No object detected")
