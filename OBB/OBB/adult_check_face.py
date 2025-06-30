'''
sudo apt install fonts-nanum
pip install scikit-image
pip install easyocr
sudo apt update
sudo apt install -y cmake build-essential python3-dev
pip install dlib
sudo apt install -y libboost-all-dev # dlib 설치중 에러날 경우 사용
pip install face_recognition
pip install easyocr
pip install pillow opencv-python scikit-image

'''


import rclpy
from rclpy.node import Node
from msgs.srv import AdultEvent

import cv2
import numpy as np
import easyocr
import re
import datetime
import time
import face_recognition
from skimage.metrics import structural_similarity as ssim
from PIL import ImageFont, ImageDraw, Image
import os

class IDVerificationNode(Node):
    def __init__(self):
        super().__init__('id_verification_node')
        self.srv = self.create_service(AdultEvent, 'adult_event', self.verify_callback)
        self.get_logger().info("✅ ID Verification Service Node is ready.")
        self.reader = easyocr.Reader(['ko'], gpu=False, verbose=False)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.roi_x, self.roi_y, self.roi_w, self.roi_h = 440, 160, 400, 250

    def verify_callback(self, request, response):
        self.get_logger().info(f"📨 Received request: {request.request}")

        while True:
            if self.run_id_ocr_stage():
                time.sleep(1)
                if self.run_face_verification_stage():
                    response.result = True
                    break
                else:
                    continue
            else:
                continue

        self.cap.release()
        # if os.path.exists("temp_roi.png"):
        #     os.remove("temp_roi.png")
        if os.path.exists("temp_id_face.png"):
            os.remove("temp_id_face.png")

        return response

    def preprocess_image(self, frame):
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(32, 32))
        enhanced = clahe.apply(frame)
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        sharpened = cv2.filter2D(enhanced, -1, kernel)
        return sharpened

    def extract_rrn(self, text):
        pattern = r'(\d{6})[-~]?(\d)'
        matches = re.findall(pattern, text)
        if matches:
            return matches[0][0], matches[0][1]
        else:
            return None, None

    def is_adult(self, birth_str, code):
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

    def draw_korean_text(self, img_cv, text, pos=(50, 50), font_size=30, color=(0, 255, 0)):
        img_pil = Image.fromarray(cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
        font = ImageFont.truetype(font_path, font_size)
        draw.text(pos, text, font=font, fill=color)
        img_cv = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        return img_cv

    def run_id_ocr_stage(self):
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        baseline_roi = gray[self.roi_y:self.roi_y + self.roi_h, self.roi_x:self.roi_x + self.roi_w]
        baseline_original = baseline_roi.copy()
        self.get_logger().info("✅ Baseline ROI 캡처 완료!")
        hold_start_time = None

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            original_frame = frame.copy()
            cv2.rectangle(frame, (self.roi_x, self.roi_y), (self.roi_x + self.roi_w, self.roi_y + self.roi_h), (0, 255, 0), 2)
            frame = self.draw_korean_text(frame, "신분증을 박스에 위치시키고 2초간 대기", (self.roi_x - 150, self.roi_y - 50), 30)

            current_roi = gray[self.roi_y:self.roi_y + self.roi_h, self.roi_x:self.roi_x + self.roi_w]
            score, _ = ssim(baseline_original, current_roi, full=True)

            if score < 0.65:
                self.get_logger().info(f"score : {score}")
                if hold_start_time is None:
                    hold_start_time = time.time()
                elif time.time() - hold_start_time >= 2.0:
                    # cv2.imwrite("temp_roi.png", current_roi)
                    cv2.imwrite("temp_id_face.png", original_frame[self.roi_y:self.roi_y + self.roi_h, self.roi_x:self.roi_x + self.roi_w])

                    # ocr_img = cv2.imread("temp_roi.png", cv2.IMREAD_GRAYSCALE)
                    ocr_img = cv2.cvtColor(current_roi, cv2.COLOR_BGR2GRAY)
                    processed = self.preprocess_image(ocr_img)
                    results = self.reader.readtext(processed, detail=0, paragraph=False)
                    full_text = ' '.join(results)

                    birth_num, code = self.extract_rrn(full_text)
                    if birth_num and code and self.is_adult(birth_num, code):
                        return True
                    else:
                        hold_start_time = None
            else:
                hold_start_time = None

            cv2.imshow("ID Verification", frame)
            if cv2.waitKey(1) == 27:
                cv2.destroyAllWindows()
                return False

    def run_face_verification_stage(self):
        id_face_encoding = None

        id_face_img = cv2.imread("temp_id_face.png")
        id_face_locations = face_recognition.face_locations(id_face_img)
        id_face_encodings = face_recognition.face_encodings(id_face_img, id_face_locations)

        if id_face_encodings:
            id_face_encoding = id_face_encodings[0]
        else:
            return False

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = self.draw_korean_text(frame, "얼굴을 인식시켜 주세요.", (400, 50), 32)
            live_face_locations = face_recognition.face_locations(frame)
            live_face_encodings = face_recognition.face_encodings(frame, live_face_locations)

            if live_face_encodings:
                result = face_recognition.compare_faces([id_face_encoding], live_face_encodings[0], tolerance=0.6)
                if result[0]:
                    frame = self.draw_korean_text(frame, "얼굴 인증 성공! 인증 완료.", (300, 400), 32, color=(0, 255, 0))
                    cv2.imshow("Face Verification", frame)
                    cv2.waitKey(2000)
                    cv2.destroyAllWindows()
                    return True
                else:
                    frame = self.draw_korean_text(frame, "얼굴 불일치. 다시 시도하세요.", (300, 400), 32, color=(0, 0, 255))

            cv2.imshow("Face Verification", frame)
            if cv2.waitKey(1) == 27:
                cv2.destroyAllWindows()
                return False

def main(args=None):
    rclpy.init(args=args)
    node = IDVerificationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
