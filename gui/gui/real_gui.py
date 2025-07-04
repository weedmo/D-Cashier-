import os
import json
import rclpy
import openai
import time
from pydub import AudioSegment
from pydub.playback import play
from rclpy.node import Node
from std_msgs.msg import Int32, String
import tkinter as tk
from threading import Thread
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

PACKAGE_PATH = get_package_share_directory('gui')
JSON_PATH = os.path.join(PACKAGE_PATH, 'resource', 'product_data.json')

def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
def latched_qos(depth: int = 1) -> QoSProfile:
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

        self.create_subscription(Int32, "/gui_command", self.gui_command_callback, latched_qos())
        self.create_subscription(String, "/class_name", self.class_name_callback, _latched_qos())

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

    def gui_command_callback(self, msg):
        code = msg.data
        if code == 0:
            self.root.withdraw()
        elif code == 1:
            self.cart_items.clear()
            self.total_price = 0
            self.root.deiconify()
            self.update_display("계산을 시작합니다")
            self.speak("계산을 시작합니다", voice="nova")
        elif code == 2:
            self.get_logger().info("🗢 웨이크워드 감지됨 → 계산 창 자동 표시")
            self.cart_items.clear()
            self.total_price = 0
            self.root.deiconify()
            self.update_display("🎤 웨이크워드 감지됨!계산을 시작합니다.")
            self.speak("계산을 시작합니다", voice="nova")
        elif code == 3:
            if not self.waiting_adult_item:
                return
            info = self.product_data.get("terra")
            if info and "price" in info:
                base_price = info["price"]
                self.speak("인증되었습니다", voice="nova")
                self.cart_items.append(("terra", 1, base_price))
                self.total_price += base_price
                display = self.generate_cart_display()
            else:
                display = "⚠️ terra의 가격 정보를 찾을 수 없습니다."
            self.waiting_adult_item = False
            self.update_display(display)
        elif code == 4:
            if not self.waiting_adult_item:
                return
            self.waiting_adult_item = False
            self.speak("인증에 실패하였습니다", voice="nova")
            self.update_display("❌ 인증에 실패했습니다. terra는 구매할 수 없습니다.")
            
        elif code == 5:  # 봉투 필요 여부 안내
            self.update_display("🛍 봉투가 필요하신가요?")
            self.speak("봉투가 필요하신가요?", voice="nova")
            
        elif code == 6:  # 봉투 추가
            self.cart_items.append(("봉투", 1, 50))
            self.total_price += 50
            self.speak("봉투를 추가했습니다.", voice="nova")
            display = self.generate_cart_display()
            self.update_display(display)

        elif code == 7:  # 봉투 추가 안 함
            self.speak("봉투를 추가하지 않았습니다.", voice="nova")
            
        elif code == 8:  # 계산 종료
            self.speak("계산을 종료합니다. 감사합니다.", voice="nova")
            self.update_display("🧾 계산이 종료되었습니다. 감사합니다.")
            time.sleep(3)  # 3초 후 창 닫기 (선택사항)
            self.root.withdraw()




    def class_name_callback(self, msg):
        name = msg.data
        info = self.product_data.get(name)
        if info is None:
            self.update_display(f"⚠️ '{name}'의 정보가 없습니다.")
            return

        if info["is_adult_only"]:
            self.waiting_adult_item = True
            self.speak("성인 인증이 필요합니다. 신분증을 제시해주세요", voice="nova")
            self.update_display("🅾️ 성인 인증이 필요합니다 🅾️(신분증을 준비해주세요)")
            return

        base_price = info["price"]
        discount = info["discount_rate"]
        is_one_plus_one = info["is_one_plus_one"]

        same_item_count = sum(1 for item in self.cart_items if item[0] == name)
        nth = same_item_count + 1
        is_odd = (nth % 2 == 1)

        if is_one_plus_one and not is_odd:
            return

        qty = 1
        price = int(base_price * (100 - discount) / 100)
        self.cart_items.append((name, qty, price))
        self.total_price += price

        if is_one_plus_one and is_odd:
            notice = f"{name}은(는) 1+1입니다하나 더 가지고 오세요"
            self.update_display(notice)
            self.speak(f"{name}은 1+1입니다. 하나 더 가지고 오세요.", voice="nova")
            def restore_display():
                time.sleep(3)
                display = self.generate_cart_display()
                self.update_display(display)
            Thread(target=restore_display, daemon=True).start()
            return

        display = self.generate_cart_display()
        self.update_display(display)
        self.speak(name, voice="nova")

    def generate_cart_display(self):
        display = "🧾 장바구니:\n"
        for item_name, item_qty, item_price in self.cart_items:
            display += f"- {item_name} ×{item_qty}: {item_price * item_qty}원\n"
        display += f"\n💰 총합: {self.total_price}원"
        return display


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
