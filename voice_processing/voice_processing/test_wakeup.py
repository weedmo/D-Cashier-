import os
import numpy as np
from scipy.signal import resample

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pyaudio

from ament_index_python.packages import get_package_share_directory
from openwakeword.model import Model

MODEL_NAME = "hello_rokey_8332_32.tflite"
PACKAGE_PATH = get_package_share_directory("voice_processing")
MODEL_PATH = os.path.join(PACKAGE_PATH, f"resource/{MODEL_NAME}")


class WakeupDetectorNode(Node):
    def __init__(self):
        super().__init__("wakeup_detector_node")

        # Publisher 설정
        self.pub = self.create_publisher(Bool, "/wakeup_detected", 10)

        # 오디오 설정
        self.buffer_size = 24000  # 0.5초 @ 48kHz
        self.stream = self._init_stream()

        # 모델 초기화
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.model = Model(wakeword_models=[MODEL_PATH])

        # 타이머 설정 (100ms 주기)
        self.timer = self.create_timer(0.1, self.detect_loop)

        self.detected = False  # 한번만 publish 하기 위한 flag
        self.get_logger().info("🎙️ Wakeup detector node started!")

    def _init_stream(self):
        p = pyaudio.PyAudio()
        return p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=48000,
            input=True,
            frames_per_buffer=self.buffer_size,
        )

    def detect_loop(self):
        if self.detected:
            return  # 이미 감지했으면 아무것도 안 함

        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))

        result = self.model.predict(audio_chunk, threshold=0.1)
        confidence = result.get(self.model_name, 0.0)
        self.get_logger().info(f"🎧 Confidence: {confidence:.4f}")

        if confidence > 0.03:
            self.get_logger().info("✅ 'hello rokey' 감지됨! → /wakeup_detected: True")
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)
            self.detected = True  # 한 번만 발행되도록 설정


def main():
    rclpy.init()
    node = WakeupDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
