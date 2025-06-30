#!/usr/bin/env python3

import os
import re

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

from jamo import h2j, j2hcj
import Levenshtein as lev


class GetKeyword(Node):
    # 직접 매칭할 계산류 명령어
    COMMAND_SYNONYMS = {
        "계산", "계산해줘", "계산해", "빨리해봐", "알아서 해", "움직여", "움직이자",
        "깨산", "게산", "개산", "갸산", "계싼", "계삳"
    }

    def __init__(self):
        super().__init__("get_keyword_node")

        # Load .env
        package_path = get_package_share_directory("voice_processing")
        load_dotenv(dotenv_path=os.path.join(package_path, "resource/.env"))
        openai_api_key = os.getenv("OPENAI_API_KEY")

        # LLM 셋업
        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.5,
            openai_api_key=openai_api_key
        )
        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - hammer, screwdriver, wrench, pos1, pos2, pos3

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 ...]
            - 도구와 위치는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "못 박는 것" → hammer)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "hammer를 pos1에 가져다 놔"  
            출력: hammer / pos1

            - 입력: "왼쪽에 있는 해머와 wrench를 pos1에 넣어줘"  
            출력: hammer wrench / pos1

            - 입력: "왼쪽에 있는 hammer를줘"  
            출력: hammer /

            - 입력: "왼쪽에 있는 못 박을 수 있는것을 줘"  
            출력: hammer /

            - 입력: "hammer는 pos2에 두고 screwdriver는 pos1에 둬"  
            출력: hammer screwdriver / pos2 pos1

            <사용자 입력>
            "{user_input}"                
        """
        self.prompt = PromptTemplate(
            input_variables=["user_input"],
            template=prompt_content
        )
        self.llm_chain = LLMChain(llm=self.llm, prompt=self.prompt)

        # STT 셋업
        self.stt = STT(openai_api_key=openai_api_key)

        # Mic 설정
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

        # 서비스 생성
        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_service = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )

    @staticmethod
    def _is_similar_to_calc(text: str, base: str = "계산", thresh: float = 0.7) -> bool:
        """자모 분해 + Levenshtein 유사도로 '계산'류 단어 탐지"""
        # 한글과 공백만 남기기
        clean = re.sub(r"[^가-힣\s]", " ", text)
        tokens = clean.split()
        base_jamo = j2hcj(h2j(base))
        for tok in tokens:
            tok_jamo = j2hcj(h2j(tok))
            if lev.ratio(base_jamo, tok_jamo) >= thresh:
                return True
        return False

    def extract_keyword(self, user_text: str) -> list[str]:
        """LLM으로부터 도구·목적지 키워드 추출"""
        resp = self.llm_chain.invoke({"user_input": user_text})
        result = resp["text"].strip()
        tools_part, dest_part = result.split("/")
        tools = tools_part.split() if tools_part else []
        # 목적지는 현재 사용 안 함; 필요 시 dest_part 처리
        return tools

    def get_keyword(self, request, response):
        # 오디오 스트림 열기
        try:
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("오디오 스트림을 열 수 없습니다 ― 디바이스 확인 필요")
            return None

        # wakeup 단어 대기
        while not self.wakeup_word.is_wakeup():
            pass

        # STT 수행
        output_message = self.stt.speech2text().strip()
        self.get_logger().info(f"💬 STT 결과: {output_message}")

        # ① 사전 매칭 ② 자모 유사도
        cond_dict = any(cmd in output_message for cmd in self.COMMAND_SYNONYMS)
        cond_jamo = self._is_similar_to_calc(output_message)

        if cond_dict or cond_jamo:
            keywords = ["계산"]
        else:
            keywords = self.extract_keyword(output_message)

        self.get_logger().info(f"💬 Keywords: {keywords}")
        # 서비스 응답
        response.success = True
        response.message = " ".join(keywords)
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
