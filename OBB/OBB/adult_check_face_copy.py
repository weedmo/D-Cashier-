import cv2
import numpy as np
import easyocr
import re
import time
import datetime
import face_recognition
from skimage.metrics import structural_similarity as ssim
from PIL import ImageFont, ImageDraw, Image
import os

def preprocess_image(frame):
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(32, 32))
    enhanced = clahe.apply(frame)
    kernel = np.array([[0, -1, 0],
                       [-1, 5, -1],
                       [0, -1, 0]])
    sharpened = cv2.filter2D(enhanced, -1, kernel)
    return sharpened

def extract_rrn(text):
    pattern = r'(\d{6})[-~]?(\d)'
    matches = re.findall(pattern, text)
    if matches:
        return matches[0][0], matches[0][1]
    else:
        return None, None

def is_adult(birth_str, code):
    try:
        year = int(birth_str[:2])
        month = int(birth_str[2:4])
        day = int(birth_str[4:6])
        code = int(code)

        if code in [1, 2]:
            year += 1900
        elif code in [3, 4]:
            year += 2000
        else:
            return False

        birth_date = datetime.datetime(year, month, day)
        today = datetime.datetime.today()

        age = today.year - birth_date.year - ((today.month, today.day) < (birth_date.month, birth_date.day))
        return age >= 20
    except:
        return False

def draw_korean_text(img_cv, text, pos=(50, 50), font_size=30, color=(0, 255, 0)):
    img_pil = Image.fromarray(cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)
    font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
    font = ImageFont.truetype(font_path, font_size)
    draw.text(pos, text, font=font, fill=color)
    img_cv = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
    return img_cv

def run_id_ocr_stage(cap, roi_x, roi_y, roi_w, roi_h, reader):
    # Baseline ROI 캡처
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    baseline_roi = gray[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
    baseline_original = baseline_roi.copy()
    print("✅ Baseline ROI 캡처 완료!")

    hold_start_time = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        original_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 0), 2)
        frame = draw_korean_text(frame, "신분증을 박스에 위치시키고 2초간 대기해주세요", (roi_x - 150, roi_y - 50), 30)

        current_roi = gray[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
        score, _ = ssim(baseline_original, current_roi, full=True)
        print(score)

        if score < 0.70:
            if hold_start_time is None:
                hold_start_time = time.time()
            else:
                duration = time.time() - hold_start_time
                if duration >= 2.0:
                    print("\n🔎 SSIM 조건 만족! OCR 및 얼굴 ROI 저장 중...")

                    cv2.imwrite("temp_roi.png", current_roi)
                    cv2.imwrite("temp_id_face.png", original_frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w])

                    ocr_img = cv2.imread("temp_roi.png", cv2.IMREAD_GRAYSCALE)
                    processed = preprocess_image(ocr_img)
                    results = reader.readtext(processed, detail=0, paragraph=False)
                    full_text = ' '.join(results)

                    birth_num, code = extract_rrn(full_text)
                    if birth_num and code:
                        print(f"✅ 주민번호 앞자리: {birth_num}, 코드: {code}")
                        if is_adult(birth_num, code):
                            print("✅ 성인입니다. 기준 얼굴 인코딩을 파일로 저장합니다.")

                            id_face_img = cv2.imread("temp_id_face.png")
                            id_face_locations = face_recognition.face_locations(id_face_img)
                            id_face_encodings = face_recognition.face_encodings(id_face_img, id_face_locations)

                            if id_face_encodings:
                                np.save("face_encoding.npy", id_face_encodings[0])
                                print("✅ 기준 얼굴 인코딩 파일(face_encoding.npy) 저장 완료.")

                                cv2.destroyAllWindows()  # OCR 창 완전 종료
                                return True
                            else:
                                print("⚠️ 신분증에서 얼굴을 감지하지 못했습니다. 프로그램 종료.")
                                cv2.destroyAllWindows()
                                return False
                        else:
                            print("❌ 미성년자입니다. 구매 불가.")
                            cv2.destroyAllWindows()
                            return False
                    else:
                        print("❌ 주민번호를 인식하지 못했습니다.")
                        cv2.destroyAllWindows()
                        return False

                    hold_start_time = None
        else:
            hold_start_time = None

        cv2.imshow("ID Verification", frame)
        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
    return False

def run_face_verification_stage(cap):
    if not os.path.exists("face_encoding.npy"):
        print("❌ face_encoding.npy 파일이 존재하지 않습니다. 먼저 OCR 단계를 수행하세요.")
        return

    id_face_encoding = np.load("face_encoding.npy")
    print("\n✅ 얼굴 인증 창 시작!")
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = draw_korean_text(frame, "얼굴을 인식시켜 주세요.", (400, 50), 32)

        elapsed = time.time() - start_time

        if elapsed >= 2.0:
            live_face_locations = face_recognition.face_locations(frame)
            live_face_encodings = face_recognition.face_encodings(frame, live_face_locations)

            if live_face_encodings:
                result = face_recognition.compare_faces([id_face_encoding], live_face_encodings[0], tolerance=0.6)
                if result[0]:
                    print("✅ 얼굴 인증 성공! 최종 인증 완료.")
                    frame = draw_korean_text(frame, "얼굴 인증 성공! 인증 완료.", (300, 400), 32, color=(0, 255, 0))
                else:
                    print("❌ 얼굴 불일치. 인증 실패.")
                    frame = draw_korean_text(frame, "얼굴 불일치. 인증 실패.", (300, 400), 32, color=(0, 0, 255))
            else:
                print("⚠️ 실시간 얼굴을 감지하지 못했습니다.")

        cv2.imshow("Face Verification", frame)
        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    roi_x, roi_y, roi_w, roi_h = 440, 160, 400, 250
    reader = easyocr.Reader(['ko'], gpu=False, verbose=False)

    ocr_success = run_id_ocr_stage(cap, roi_x, roi_y, roi_w, roi_h, reader)

    time.sleep(1)

    if ocr_success:
        run_face_verification_stage(cap)
    else:
        print("❌ 얼굴 인증 단계로 진입하지 못했습니다.")

    cap.release()

    # 임시 파일 삭제
    if os.path.exists("temp_roi.png"):
        os.remove("temp_roi.png")
    if os.path.exists("temp_id_face.png"):
        os.remove("temp_id_face.png")

if __name__ == "__main__":
    main()
