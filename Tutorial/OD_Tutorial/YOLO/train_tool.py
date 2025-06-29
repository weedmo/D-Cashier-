from ultralytics import YOLO

model = YOLO("yolo11n.pt")

model.train(
    data="/home/rokey/Downloads/tools/data.yaml",
    epochs=50,
    imgsz=640,
    batch=32,
    device='0',
    workers=4,
    name='yolo11n-tools',
    patience=15  # 15 epoch 동안 val_loss 향상 없으면 종료
)
