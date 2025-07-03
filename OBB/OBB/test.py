import cv2

from ultralytics import YOLO

# Load the YOLO model
model = YOLO("/home/rokey/rokey_ws/src/OBB/resource/obj_detect_obb.pt")

# Open the video file
video_path = 6
cap = cv2.VideoCapture(video_path)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLO inference on the frame
        results = model(frame)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        for res in results:
            for poly, score, label in zip(
                res.obb.xyxyxyxy.tolist(),  # [x1,y1,...,x4,y4]
                res.obb.conf.tolist(),
                res.obb.cls.tolist(),
            ):
                print(poly)

        # Display the annotated frame
        cv2.imshow("YOLO Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()