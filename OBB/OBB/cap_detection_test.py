import cv2
import numpy as np
from ultralytics import YOLO

# 이미지 로드
image_path = 'OBB/resource/Bacchus_img_12.jpg'
image = cv2.imread(image_path)
if image is None:
    raise ValueError("이미지를 불러올 수 없습니다.")

# YOLO OBB 모델
model = YOLO("OBB/resource/obj_detect_obb.pt")
results = model(image_path)

draw_image = image.copy()

for poly in results[0].obb.xyxyxyxy.cpu().numpy():
    pts = poly.reshape(4, 2).astype(np.float32)

    # ✅ 항상 cv2.minAreaRect를 이용해 회전된 bounding box 추출
    rect = cv2.minAreaRect(pts)
    (center_x, center_y), (w, h), angle = rect

    if w < h:
        angle_vec_angle = angle
    else:
        angle_vec_angle = angle + 90

    theta = np.deg2rad(angle_vec_angle)
    short_vec_norm = np.array([np.cos(theta), np.sin(theta)])

    # ✅ 중심선과 수직 벡터
    normal_vec = np.array([-short_vec_norm[1], short_vec_norm[0]])

    # 중심점
    center = np.array([center_x, center_y])

    # polygon mask
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [pts.astype(np.int32)], 255)

    # polygon 내부 추출
    roi_image = cv2.bitwise_and(image, image, mask=mask)

    # HSV 변환
    hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 첫 번째 vertex 기준 벡터
    ref_point = pts[0]
    ref_vec = ref_point - center
    dot_ref = np.dot(ref_vec, normal_vec)
    print(type(yellow_mask))
    # 노란색 픽셀 좌표
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

    # 출력
    print(f"첫번째 좌표에 가까운 쪽 노란색 픽셀 개수: {near_count}")
    print(f"첫번째 좌표에서 먼 쪽 노란색 픽셀 개수: {far_count}")

    # polygon 시각화
    cv2.polylines(draw_image, [pts.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)

    # 첫 번째 vertex 시각화 (파란 원)
    first_pt = (int(ref_point[0]), int(ref_point[1]))
    cv2.circle(draw_image, first_pt, 5, (255, 0, 0), -1)

    # ✅ 모든 vertex 번호 표시
    for idx, pt in enumerate(pts):
        cv2.putText(draw_image, f"{idx}", (int(pt[0]), int(pt[1]) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # 중심선 그리기 (짧은 edge 기준)
    line_length = 150
    pt1 = (int(center[0] - short_vec_norm[0] * line_length), int(center[1] - short_vec_norm[1] * line_length))
    pt2 = (int(center[0] + short_vec_norm[0] * line_length), int(center[1] + short_vec_norm[1] * line_length))
    cv2.line(draw_image, pt1, pt2, (0, 0, 255), 2)

    # 텍스트 표시
    cv2.putText(draw_image, f"Near (1st pt): {near_count}", (int(center[0]-100), int(center[1]+20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv2.putText(draw_image, f"Far (1st pt): {far_count}", (int(center[0]-100), int(center[1]-20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # ✅ Near / Far mask 시각화
    near_vis = cv2.bitwise_and(roi_image, roi_image, mask=near_mask)
    far_vis = cv2.bitwise_and(roi_image, roi_image, mask=far_mask)

    cv2.imshow("Yellow Mask (Near Side)", near_vis)
    cv2.imshow("Yellow Mask (Far Side)", far_vis)

# 최종 결과 출력
cv2.imshow("OBB with Yellow Area (Final Separate Masks)", draw_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
