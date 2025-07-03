#!/usr/bin/env python3
import os
import rclpy
import pyaudio
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from dotenv import load_dotenv

from ament_index_python.packages import get_package_share_directory

from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.STT import STT


# ─────────────────────────────
def main_prompt_template() -> str:
    return """
다음 사용자 문장을 보고 어떤 의도인지 판단하여 아래 규칙에 따라 단어 하나로만 응답하세요.

<규칙>
1. 사용자가 계산을 요청하거나 어떤 동작을 시작하려는 의도가 있을 경우 → "계산"이라고 출력하세요.
   - 예시 문장: 계산해줘, 해줘, 움직여봐, 시작해, 실행해 등
   - 계산과 비슷한 발음(게산, 깨산, 개산 등)도 모두 계산으로 간주하세요.

2. 사용자가 멈추거나 잠깐 기다리자는 의도를 보이면 → "정지"라고 출력하세요.
   - 예시 문장: 멈춰, 스톱, 잠깐만, 잠시만 기다려 등

3. 사용자가 긍정하거나 동의하는 문장을 말한 경우 → "네"라고 출력하세요.
   - 예시 문장: 네, 알겠어, 그래, 좋아, 응 등

4. 만약 다음 물품 중 하나가 포함되어 있다면 해당 **물품 이름만** 출력하세요:
   - 박카스, 참크레커, 에너지바, 자유시간, 오레오, 테라, 영양갱
   - 단, 발음이 이상하더라도 유사하면 포함된 것으로 간주합니다.
   - 예시 문장: 박카스 빼줘 → "박카스", 테라 취소해 → "테라"

5. 위 1~4 규칙에 해당하지 않더라도, 반드시 세 가지 중 가장 가까운 의미나 물품 이름으로 출력하세요.

<출력 형식>
- 반드시 아래 중 하나 또는 물품 이름 중 하나만 출력:
  - "계산", "정지", "네"
  - 또는: "박카스", "참크레커", "에너지바", "자유시간", "오레오", "테라", "영양갱"

<사용자 입력>
"{user_input}"
"""

def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
# ─────────────────────────────
class KeywordPublisher(Node):
    """STT + LLM 결과를 /keyword 토픽으로 퍼블리시"""

    def __init__(self):
        super().__init__("keyword_publisher")

        # ── OpenAI 키 로드 ─────────────────────
        pkg_path = get_package_share_directory("voice_processing")
        load_dotenv(dotenv_path=os.path.join(pkg_path, "resource", ".env"))
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            self.get_logger().fatal("OPENAI_API_KEY 환경변수가 설정되지 않았습니다.")
            raise RuntimeError("Missing OpenAI API key")

        # ── LLM 체인 준비 ─────────────────────
        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.0,
            openai_api_key=openai_api_key,
        )
        prompt = PromptTemplate(
            input_variables=["user_input"],
            template=main_prompt_template(),
        )
        self.chain = LLMChain(llm=self.llm, prompt=prompt)

        # ── STT & 마이크 ──────────────────────
        self.stt = STT(openai_api_key=openai_api_key)
        mic_cfg = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_cfg)
        self.wakeup_word = WakeupWord(mic_cfg.buffer_size)

        # ── 퍼블리셔 (latched) ─────────────────
        self.pub = self.create_publisher(String, "/keyword", _latched_qos())

        # ── 타이머: 0.2 Hz(5 초)마다 실행 ───────
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("💬 KeywordPublisher 노드 준비 완료 — /keyword 퍼블리시 중")

    # ────────────────────────────────────────
    def timer_callback(self):
        """5 초 주기로:
        1) 마이크 스트림 열기
        2) 웨이크업워드 기다리기
        3) STT → LLM → 의도 추정
        4) /keyword 퍼블리시"""
        try:
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("❌ 오디오 스트림을 열 수 없습니다.")
            return

        # 웨이크업워드 대기
        while not self.wakeup_word.is_wakeup() and rclpy.ok():
            pass

        # 음성 → 텍스트
        user_text = self.stt.speech2text().strip()
        self.get_logger().info(f"🎙️ STT 결과: {user_text}")

        # 텍스트 → 의도
        intent = self.chain.invoke({"user_input": user_text})["text"].strip()
        self.get_logger().info(f"➡️  의도: {intent}")

        # 퍼블리시
        msg = String(data=intent)
        self.pub.publish(msg)
        self.get_logger().info("✅ /keyword 퍼블리시 완료")

        # 스트림 닫기
        self.mic_controller.close_stream()


# ─────────────────────────────
def main():
    rclpy.init()
    node = KeywordPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
