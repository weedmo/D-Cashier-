import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from msgs.srv import AdultEvent
from std_msgs.msg import Int32 # 1이면 인증성공, 0이면 인증실패

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
        self.srv = self.create_service(AdultEvent, '/adult_event', self.verify_callback)
        self.get_logger().info("✅ ID Verification Service Node is ready.")
        self.reader = easyocr.Reader(['ko'], gpu=False, verbose=False)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("❌ 카메라를 열 수 없습니다. 종료합니다.")
            exit(1)

        self.roi_x, self.roi_y, self.roi_w, self.roi_h = 440, 160, 400, 250

    def verify_callback(self, request, response):
        self.get_logger().info(f"📨 Received request: {request.class_name}")

        response.state_adult_event = False  # 기본값

        ocr_result = self.run_id_ocr_stage()

        if ocr_result == "success":
            time.sleep(5)
            face_success = self.run_face_verification_stage()
            if face_success:
                response.state_adult_event = True
                self.get_logger().info("✅ 모든 인증 단계 완료. 성인 인증 성공.")
            else:
                response.state_adult_event = False
                self.get_logger().warn("⚠️ 얼굴 인증 실패. 응답 후 다음 요청 대기.")
            return response

        elif ocr_result == "minor":
            self.get_logger().warn("⚠️ 미성년자로 판별되었습니다. 인증 실패.")
            return response

        else:
            self.get_logger().warn("⚠️ OCR 단계 실패. 주민번호 추출 실패 등.")
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
        if not ret:
            self.get_logger().error("❌ 카메라 프레임을 읽을 수 없습니다. 종료합니다.")
            return "fail"

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        baseline_roi = gray[self.roi_y:self.roi_y + self.roi_h, self.roi_x:self.roi_x + self.roi_w]
        baseline_original = baseline_roi.copy()
        hold_start_time = None

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            original_frame = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            current_roi = gray[self.roi_y:self.roi_y + self.roi_h, self.roi_x:self.roi_x + self.roi_w]
            score, _ = ssim(baseline_original, current_roi, full=True)

            if score < 0.68:
                if hold_start_time is None:
                    hold_start_time = time.time()
                elif time.time() - hold_start_time >= 2.0:
                    cv2.imwrite("temp_roi.png", current_roi)
                    cv2.imwrite("temp_id_face.png", original_frame[self.roi_y:self.roi_y + self.roi_h, self.roi_x:self.roi_x + self.roi_w])

                    ocr_img = cv2.imread("temp_roi.png", cv2.IMREAD_GRAYSCALE)
                    processed = self.preprocess_image(ocr_img)
                    results = self.reader.readtext(processed, detail=0, paragraph=False)
                    full_text = ' '.join(results)

                    birth_num, code = self.extract_rrn(full_text)

                    if not (birth_num and code):
                        self.get_logger().error("❌ 주민번호 추출 실패. OCR 재시도 중...")
                        hold_start_time = None
                        continue

                    if self.is_adult(birth_num, code):
                        self.get_logger().info("✅ 성인입니다. 얼굴 인증을 위해 준비할 시간을 제공합니다.")
                        frame = self.draw_korean_text(frame, "성인, 얼굴 인증을 위해 준비할 시간을 제공", (100, 200), 32, color=(0, 255, 0))
                        cv2.imshow("ID Verification", frame)
                        cv2.waitKey(3000)  # 메시지를 3초간 띄움
                        cv2.destroyAllWindows()

                        # 카메라 종료
                        self.cap.release()
                        time.sleep(5)  # 신분증을 치울 시간

                        # 얼굴 인증 단계 전에 다시 카메라 열기
                        self.cap = cv2.VideoCapture(0)
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                        # time.sleep(5)
                        # cv2.destroyAllWindows()  # ✅ OCR 창 닫기
                        return "success"
                    else:
                        self.get_logger().warn("⚠️ 미성년자로 판별되었습니다. 인증 실패.")
                        cv2.destroyAllWindows()  # ✅ OCR 창 닫기
                        return "minor"
            else:
                hold_start_time = None

            display_frame = frame.copy()
            cv2.rectangle(display_frame, (self.roi_x, self.roi_y), (self.roi_x + self.roi_w, self.roi_y + self.roi_h), (0, 255, 0), 2)
            display_frame = self.draw_korean_text(display_frame, "신분증을 박스에 위치시키고 대기 중...", (self.roi_x - 150, self.roi_y - 50), 30)
            cv2.imshow("ID Verification", display_frame)
            cv2.waitKey(30)

    def run_face_verification_stage(self):
        id_face_img = cv2.imread("temp_id_face.png")
        if id_face_img is None:
            self.get_logger().error("❌ temp_id_face.png 파일을 읽지 못했습니다.")
            return False

        id_face_locations = face_recognition.face_locations(id_face_img)
        id_face_encodings = face_recognition.face_encodings(id_face_img, id_face_locations)

        if not id_face_encodings:
            self.get_logger().error("❌ 신분증에서 얼굴을 찾을 수 없습니다. 인증 실패.")
            return False

        id_face_encoding = id_face_encodings[0]

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

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
                    self.get_logger().warn("⚠️ 얼굴 불일치. 인증 실패. 응답 후 다음 요청 대기.")
                    frame = self.draw_korean_text(frame, "얼굴 불일치. 인증 실패.", (300, 400), 32, color=(0, 0, 255))
                    cv2.imshow("Face Verification", frame)
                    cv2.waitKey(2000)
                    cv2.destroyAllWindows()
                    return False

            display_frame = self.draw_korean_text(frame, "얼굴을 인식 중...", (400, 50), 32)
            cv2.imshow("Face Verification", display_frame)
            cv2.waitKey(30)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IDVerificationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()