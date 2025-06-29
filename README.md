# base
voice langchain # 🤖 Voice-Based Pick and Place (Doosan M0609)

이 예제는 음성 명령을 통해 두산 협동로봇 M0609이 물체를 탐지하고 집는 과정을 시연하는 데모입니다.

## 🧩 구성 요소

- **Wake Word**: `"hello rokey"` → STT 시작 트리거
- **STT + LangChain(GPT)**: 자연어 명령을 처리하고 JSON ID와 매칭
- **YOLOv8**: 해당 물체를 영상에서 탐지
- **Doosan M0609**: YOLO 결과 좌표 기반으로 픽 앤 플레이스 수행

## 📁 파일 구성

```
voice_pick_demo/
├── stt_node.py                # Wake word + 음성 인식
├── langchain_parser.py        # GPT 처리 및 ID 매핑
├── detect_yolo.py             # YOLOv8을 통한 물체 탐지
├── doosan_pick.py             # 로봇으로 좌표 이동 및 집기
├── items.json                 # {"망치": "hammer", "item_id": 2} 형태
└── README.md
```

## ▶️ 동작 예시

```
사용자: "hello rokey"
→ STT: "망치 가져와"
→ GPT 결과: { "item": "hammer", "id": 2 }
→ YOLO: hammer 인식 → 좌표 추출
→ Doosan M0609 → 물체로 이동 및 집기 수행
```

> 해당 예제는 데모 목적의 참고 코드이며 실제 환경에 맞춘 보정 및 안전제어 로직이 필요합니다.
