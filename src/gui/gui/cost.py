import tkinter as tk
import threading
import sounddevice as sd
import numpy as np
import whisper
import tempfile
import os
import queue
import scipy.io.wavfile as wav

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

TRIGGER_KEYWORD = "계산해줘"

class DoosanMarketApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DOOSAN MARKET")
        self.geometry("400x400")

        self.center_frame = tk.Frame(self)
        self.center_frame.place(relx=0.5, rely=0.3, anchor="center")

        self.label = tk.Label(self.center_frame, text="계산하기", font=("Arial", 24))
        self.label.pack(pady=10)

        self.result_text = tk.Text(self, height=10, width=40)
        self.result_text.place(relx=0.5, rely=0.65, anchor="center")

        self.button = tk.Button(self, text="🎤 말하기", font=("Arial", 14), command=self.start_listening)
        self.button.place(relx=0.5, rely=0.85, anchor="center")

        self.model = whisper.load_model("base")

        # ROS 초기화
        rclpy.init()
        self.node = rclpy.create_node("gui_client")
        self.cli = self.node.create_client(Trigger, "get_product_info")

        # 서비스 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("⏳ 서비스 대기 중...")

    def start_listening(self):
        threading.Thread(target=self.record_and_transcribe, daemon=True).start()

    def record_and_transcribe(self):
        samplerate = 16000
        duration = 5
        print("🎙️ 5초 동안 말하세요...")

        audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype="int16")
        sd.wait()

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            path = f.name
            wav.write(path, samplerate, audio)

        try:
            result = self.model.transcribe(path, language="ko")
            text = result["text"].strip()
            print("🎧 인식된 문장:", text)

            if TRIGGER_KEYWORD in text.replace(" ", ""):
                print("✅ '계산해줘' 감지됨")
                self.label.config(text="계산을 시작합니다!")

                # 서비스 요청
                self.call_service()
        except Exception as e:
            print("❌ 인식 오류:", e)
        finally:
            os.remove(path)

    def call_service(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result().success:
            self.result_text.delete("1.0", tk.END)
            self.result_text.insert(tk.END, future.result().message)
            print("🧾 상품 정보 수신 완료")
        else:
            self.result_text.insert(tk.END, f"❌ 실패: {future.result().message}")

if __name__ == "__main__":
    app = DoosanMarketApp()
    app.mainloop()
    rclpy.shutdown()
