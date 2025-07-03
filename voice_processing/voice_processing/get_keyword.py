#!/usr/bin/env python3

import os
import rclpy
import pyaudio
from rclpy.node import Node
from dotenv import load_dotenv

from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.STT import STT


def main_prompt_template() -> str:
    return """
다음 사용자 문장을 보고 어떤 의도인지 판단하여 아래 규칙에 따라 단어 하나로만 응답하세요.

<규칙>
1. 사용자가 계산을 요청하거나 어떤 동작을 시작하려는 의도가 있을 경우 → "계산"이라고 출력하세요.
   - 예시 문장: 계산해줘, 해줘, 움직여봐, 시작해, 실행해 등
   - 계산과 비슷한 발음(게산, 깨산 등)도 모두 계산으로 간주하세요.

2. 사용자가 멈추거나 잠깐 기다리자는 의도를 보이면 → "정지"라고 출력하세요.
   - 예시 문장: 멈춰, 스톱, 잠깐만, 잠시만 기다려 등

3. 사용자가 긍정하거나 동의하는 문장을 말한 경우 → "네"라고 출력하세요.
   - 예시 문장: 네, 알겠어, 그래, 좋아, 응 등

4. 그 외 어떤 경우든 세 가지 중 가장 가까운 의미로 매칭하세요.

<출력 형식>
- 반드시 아래 중 하나만 출력: "계산", "정지", "네"

<사용자 입력>
"{user_input}"
"""

class GetKeyword(Node):
    def __init__(self):
        super().__init__('get_keyword_node')

        # Load environment variables for OpenAI API
        pkg_path = get_package_share_directory('voice_processing')
        load_dotenv(dotenv_path=os.path.join(pkg_path, 'resource', '.env'))
        openai_api_key = os.getenv('OPENAI_API_KEY')
        if not openai_api_key:
            self.get_logger().error('OPENAI_API_KEY 환경변수가 설정되지 않았습니다.')
            raise RuntimeError('Missing OpenAI API key')

        # Initialize LLM with custom prompt
        self.llm = ChatOpenAI(
            model='gpt-4o',
            temperature=0.0,
            openai_api_key=openai_api_key
        )
        self.prompt = PromptTemplate(
            input_variables=['user_input'],
            template=main_prompt_template()
        )
        self.chain = LLMChain(llm=self.llm, prompt=self.prompt)

        # Initialize STT
        self.stt = STT(openai_api_key=openai_api_key)

        # Setup microphone controller and wakeup word
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        # Create Trigger service for keyword extraction
        self.get_logger().info('GetKeyword node initialized.')
        self.create_service(Trigger, 'get_keyword', self.get_keyword_callback)
        self.get_logger().info("Service 'get_keyword' is ready.")

    def get_keyword_callback(self, request, response):
        # Open audio stream and wait for wakeup
        try:
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error('오디오 스트림을 열 수 없습니다.')
            response.success = False
            response.message = ''
            return response

        while not self.wakeup_word.is_wakeup():
            pass

        # Perform STT
        user_text = self.stt.speech2text().strip()
        self.get_logger().info(f'STT 결과: {user_text}')

        # Invoke LLM to determine intent
        result = self.chain.invoke({'user_input': user_text})['text'].strip()
        self.get_logger().info(f'의도: {result}')

        # Return as Trigger response
        response.success = True
        response.message = result
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

