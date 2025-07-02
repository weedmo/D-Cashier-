import cv2

# 콜백 함수 정의
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked coordinates: x={x}, y={y}")
        # 선택적으로 이미지에 점을 찍을 수도 있음
        cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
        cv2.imshow("Image", img)

# 이미지 불러오기
image_path = "resource/back.jpg"  # 불러올 이미지 경로로 변경하세요
img = cv2.imread(image_path)

if img is None:
    raise ValueError("이미지를 불러올 수 없습니다. 경로를 확인하세요.")

cv2.imshow("Image", img)

# 마우스 콜백 등록
cv2.setMouseCallback("Image", click_event)

# ESC를 누를 때까지 대기
while True:
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break

cv2.destroyAllWindows()
