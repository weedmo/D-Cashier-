import os
import json
import rclpy
import openai
from pydub import AudioSegment
from pydub.playback import play
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
import tkinter as tk
from threading import Thread
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from dotenv import load_dotenv

PACKAGE_PATH = get_package_share_directory('gui')
JSON_PATH = os.path.join(PACKAGE_PATH, 'resource', 'product_data.json')

def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )

class CashierGUI(Node):
    def __init__(self):
        super().__init__('cashier_gui_node')
       
        self.product_data = self.load_product_data(JSON_PATH)
        self.cart_items = []
        self.total_price = 0
        self.waiting_adult_item = False

        self.root = tk.Tk()
        self.root.title("🍎 D-Cashier")
        self.root.geometry("500x450")
        self.root.configure(bg="#f8f8f8")
        self.root.withdraw()

        self.title_label = tk.Label(self.root, text="🍎 D-Cashier", font=("Arial", 20, "bold"), bg="#f8f8f8")
        self.title_label.pack(pady=(20, 10))

        self.display = tk.Text(self.root, font=("Arial", 13), height=15, width=52, bg="#ffffff", relief="ridge", borderwidth=2)
        self.display.pack(padx=20, pady=10)
        self.display.config(state="disabled")

        btn_frame = tk.Frame(self.root, bg="#f8f8f8")
        btn_frame.pack(pady=10)

        self.close_btn = tk.Button(btn_frame, text="닫기", command=self.close_window, bg="#e74c3c", fg="white", width=15)
        self.close_btn.grid(row=0, column=0, padx=10)

        # ROS 2 Subscribers
        self.create_subscription(Bool, "/prompt_pub", self.prompt_callback, _latched_qos())
        self.create_subscription(String, "/class_name", self.class_name_callback, _latched_qos())
        self.create_subscription(Bool, "/wakeup_detected", self.wakeup_callback, _latched_qos())
        self.create_subscription(Int32, "/adult_check_result", self.adult_result_callback, _latched_qos())

    def load_product_data(self, path):
        if not os.path.exists(path):
            self.get_logger().warn(f"❗ JSON 파일이 존재하지 않습니다: {path}")
            return {}
        with open(path, 'r') as f:
            try:
                return json.load(f)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"❌ JSON 파싱 오류: {e}")
                return {}

    def update_display(self, text: str):
        self.display.config(state="normal")
        self.display.delete("1.0", tk.END)
        self.display.insert(tk.END, text)
        self.display.config(state="disabled")

    def speak(self, text, voice="nova"):
        try:
            response = openai.audio.speech.create(
                model="tts-1",
                voice=voice,
                input=text
            )
            mp3_path = "/tmp/tts_output.mp3"
            with open(mp3_path, "wb") as f:
                f.write(response.read())
            sound = AudioSegment.from_file(mp3_path, format="mp3")
            play(sound)
        except Exception as e:
            self.get_logger().error(f"TTS 실행 실패: {e}")

    def prompt_callback(self, msg):
        if msg.data:
            self.cart_items.clear()
            self.total_price = 0
            self.root.deiconify()
            self.update_display("계산을 시작합니다")
            self.speak("계산을 시작합니다", voice="nova")
        else:
            self.root.withdraw()

    def class_name_callback(self, msg):
        name = msg.data

        # ① 제품 정보(dict) 가져오기
        info = self.product_data.get(name)      # None 이면 미등록
        if info is None:
            display = f"⚠️ '{name}'의 정보가 없습니다."
            self.update_display(display)
            return

        # ② 성인 상품 여부
        if info["is_adult_only"]:
            display = "🅾️ 성인 인증이 필요합니다 🅾️\n(신분증을 준비해주세요)"
            self.waiting_adult_item = True
            self.speak("성인 인증이 필요합니다. 신분증을 제시해주세요", voice="nova")
            self.update_display(display)
            return

        # ③ 할인·1+1 계산
        base_price = info["price"]
        discount   = info["discount_rate"]       # %
        qty        = 2 if info["is_one_plus_one"] else 1
        price      = int(base_price * (100 - discount) / 100)

        # ④ 장바구니 업데이트
        self.cart_items.append((name, qty, price))
        self.total_price += price * qty

        # ⑤ 출력
        display = f"📦 최근 인식: {name} ({price}원, 수량 {qty})\n\n🧾 장바구니:\n"
        for item_name, item_qty, item_price in self.cart_items:
            display += f" - {item_name} ×{item_qty}: {item_price * item_qty}원\n"
        display += f"\n💰 총합: {self.total_price}원"
        self.update_display(display)
        self.speak(name, voice="nova")


    def wakeup_callback(self, msg):
        if msg.data:
            self.get_logger().info("🗢 웨이크워드 감지됨 → 계산 창 자동 표시")
            self.cart_items.clear()
            self.total_price = 0
            self.root.deiconify()
            self.update_display("🎤 웨이크워드 감지됨!\n계산을 시작합니다.")
            self.speak("계산을 시작합니다", voice="nova")

    def adult_result_callback(self, msg):
        if not self.waiting_adult_item:
            return

        if msg.data == 1:
            price = self.product_data.get("terra", None)
            self.speak("인증되었습니다", voice="nova")
            if price is not None:
                self.cart_items.append(("terra", price))
                self.total_price += price
                display = f"✅ 성인 인증 성공!\n📦 terra ({price}원) 장바구니에 추가됨.\n\n🧾 장바구니:\n"
                for item_name, item_price in self.cart_items:
                    display += f" - {item_name}: {item_price}원\n"
                display += f"\n💰 총합: {self.total_price}원"
            else:
                display = "⚠️ terra의 가격 정보를 찾을 수 없습니다."
        else:
            display = "❌ 인증에 실패했습니다. terra는 구매할 수 없습니다."
            self.speak("인증에 실패하였습니다", voice="nova")

        self.waiting_adult_item = False
        self.update_display(display)

    def close_window(self):
        self.root.withdraw()


def main():
    rclpy.init()    
    gui_node = CashierGUI()
    def ros_spin():
        rclpy.spin(gui_node)

    Thread(target=ros_spin, daemon=True).start()
    gui_node.root.mainloop()
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
