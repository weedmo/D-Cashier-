import cv2
import numpy as np

class PolygonProcessor:
    """
    배경 이미지와 현재 이미지를 비교하여 움직이는 객체(또는 변화 영역)를 검출하고,
    해당 객체의 최소 외접 사각형(minAreaRect)을 구해 중심 좌표, 크기, yaw(회전 각도), vertex 정보를 시각화 및 반환하는 클래스.
    """

    def __init__(self, background_image):
        """
        Args:
            background_image: 배경 이미지 (참조 이미지)

        Raises:
            ValueError: 입력 이미지가 None일 때 예외 발생
        """
        if background_image is None:
            raise ValueError("Background image is None")
        self.background = background_image
        self.roi_vertices = [(48, 3), (526, 6), (529, 416), (50, 413)]  # ROI polygon vertices

    def compute_yaw_from_long_edge(self, vertices):
        """
        polygon vertex 중 가장 긴 edge 방향을 기준으로 yaw(회전 각도) 계산.
        Args:
            vertices: polygon vertex 리스트 [[x1, y1], [x2, y2], ...]
        Returns:
            yaw (radian): -pi/2 ~ pi/2 범위로 보정된 회전 각도
        """
        # edge vector 계산
        edges = [
            (vertices[1][0] - vertices[0][0], vertices[1][1] - vertices[0][1]),
            (vertices[2][0] - vertices[1][0], vertices[2][1] - vertices[1][1]),
            (vertices[3][0] - vertices[2][0], vertices[3][1] - vertices[2][1]),
            (vertices[0][0] - vertices[3][0], vertices[0][1] - vertices[3][1])
        ]
        lengths = [np.hypot(dx, dy) for dx, dy in edges]  # edge 길이 계산
        max_idx = np.argmax(lengths)  # 가장 긴 edge index
        dx, dy = edges[max_idx]

        yaw = np.arctan2(dy, dx)  # yaw 계산

        # 각도 범위를 [-pi/2, pi/2]로 보정
        if yaw > np.pi / 2:
            yaw -= np.pi
        elif yaw < -np.pi / 2:
            yaw += np.pi

        return yaw

    def is_counter_clockwise(self, vertices):
        """
        polygon vertex 순서가 반시계 방향인지 확인 (signed area 이용)
        Args:
            vertices: polygon vertex 리스트
        Returns:
            bool: True (반시계), False (시계)
        """
        sum = 0.0
        n = len(vertices)
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]
            sum += (x2 - x1) * (y2 + y1)
        return sum < 0

    def process_and_visualize(self, current_image, show=True, window_name="Polygon Result"):
        """
        현재 이미지를 처리하고, 결과 이미지를 창에 띄우는 함수.
        Args:
            current_image: 현재 프레임 이미지
            show: 결과 표시 여부
            window_name: 표시할 창 이름
        Returns:
            처리 결과 dict (box, poly_img, vertices, yaw)
        """
        result = self.process(current_image)
        if result and show:
            cv2.imshow(window_name, result["poly_img"])
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return result

    def process(self, current_image):
        """
        객체 검출 및 polygon vertex 기반 정보 추출.
        Args:
            current_image: 현재 이미지
        Returns:
            dict: {
                "box": (cx, cy, w, h, yaw),
                "poly_img": 시각화 이미지,
                "vertices": vertex 리스트,
                "yaw": yaw (radian)
            }
            또는 None (검출 실패 시)
        """
        if current_image is None:
            raise ValueError("Current image is None")

        # 배경과 shape이 다르면 resize
        if self.background.shape != current_image.shape:
            current_image = cv2.resize(current_image, (self.background.shape[1], self.background.shape[0]))

        # ROI polygon 마스크 생성
        mask = np.zeros(self.background.shape[:2], dtype=np.uint8)
        if self.roi_vertices is not None:
            roi_pts = np.array(self.roi_vertices, dtype=np.int32)
            cv2.fillPoly(mask, [roi_pts], 255)

        # 배경과 현재 이미지 차이 계산
        diff = cv2.absdiff(current_image, self.background)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)  # 변화 영역 이진화

        # ROI 적용
        if self.roi_vertices is not None:
            thresh = cv2.bitwise_and(thresh, mask)

        # Morphological 연산으로 noise 제거 및 영역 정리
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # 외곽선 찾기
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # 시각화용 이미지 복사
        poly_img = current_image.copy()

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1500:  # 작은 contour 무시
                continue

            # 최소 외접 사각형
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (w, h), angle = rect
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            vertices = [[int(x), int(y)] for x, y in box]

            # vertex 순서 반시계 방향 보정
            if not self.is_counter_clockwise(vertices):
                vertices = vertices[::-1]

            # yaw 계산
            yaw = self.compute_yaw_from_long_edge(vertices)

            # 시각화: box, 중심점, vertex
            cv2.drawContours(poly_img, [box], 0, (0, 255, 0), 2)
            cv2.circle(poly_img, (int(cx), int(cy)), 5, (0, 0, 255), -1)

            for i, (px, py) in enumerate(vertices):
                cv2.circle(poly_img, (px, py), 5, (255, 0, 255), -1)
                text = f"({px},{py})"
                cv2.putText(poly_img, text, (px + 5, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

            # yaw 텍스트 표시
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
    # 배경 이미지 로드
    background = cv2.imread("/home/jsbae/ros2_ws/images5/back_.jpg")
    current = cv2.imread("/home/jsbae/ros2_ws/images5/harribo_.jpg")

    # ROI polygon 정의
    roi_vertices = [(40, 14), (518, 5), (522, 428), (48, 430)]

    processor = PolygonProcessor(background)

    # 현재 이미지 처리 및 시각화
    result = processor.process_and_visualize(current)

    if result:
        print(f"box: {result['box']}")
        print(f"Yaw (deg): {np.degrees(result['yaw']):.2f}")
        print(f"Vertices: {result['vertices']}")
    else:
        print("No object detected")
