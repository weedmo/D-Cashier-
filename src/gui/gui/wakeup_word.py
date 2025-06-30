import numpy as np
import openwakeword
from openwakeword.model import Model
from scipy.signal import resample
import threading
import tkinter as tk
import MicController
import os  # 추가
import subprocess

# 모델 경로를 현재 파일 기준으로 절대 경로로 지정
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_NAME = "hello_rokey_8332_32.tflite"
MODEL_PATH = os.path.join(CURRENT_DIR, MODEL_NAME)


class WakeupWord:
    def __init__(self, buffer_size):
        openwakeword.utils.download_models()
        self.model = None
        self.model_name = os.path.splitext(os.path.basename(MODEL_NAME))[0]
        self.stream = None
        self.buffer_size = buffer_size

    def is_wakeup(self):
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs[self.model_name]
        print("confidence: ", confidence)
        if confidence > 0.001:
            print("Wakeword detected!")
            return True
        return False

    def set_stream(self, stream):
        self.model = Model(wakeword_models=[MODEL_PATH])  # 경로로 전달
        self.stream = stream


# ---------- Tkinter GUI ---------- #
def launch_gui():
    root = tk.Tk()
    root.title("DOOSAN MARKET")
    root.geometry("400x200")

    frame = tk.Frame(root)
    frame.place(relx=0.5, rely=0.5, anchor="center")

    label = tk.Label(frame, text="Say 'Hello Rokey'", font=("Arial", 20))
    label.pack()

    root.mainloop()


# ---------- Main Program ---------- #
if __name__ == "__main__":
    # GUI를 별도 스레드에서 실행
    gui_thread = threading.Thread(target=launch_gui)
    gui_thread.daemon = True
    gui_thread.start()

    # 마이크 스트림 및 웨이크워드 감지 시작
    Mic = MicController.MicController()
    Mic.open_stream()

    wakeup = WakeupWord(Mic.config.buffer_size)
    wakeup.set_stream(Mic.stream)

    while not wakeup.is_wakeup():
        pass

    print("▶ Wake word activated!")

    # cost.py 실행
    subprocess.Popen(["python3", os.path.join(CURRENT_DIR, "cost.py")])