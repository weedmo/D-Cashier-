<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/OpenCV-4.x-green?logo=opencv" />
  <img src="https://img.shields.io/badge/EasyOCR-1.x-lightgrey?logo=python" />
  <img src="https://img.shields.io/badge/YOLOv11n--OBB-Custom-orange?logo=darkreader" />
  <img src="https://img.shields.io/badge/Doosan--Manipulator-M0609-critical?logo=robotframework" />
  <img src="https://img.shields.io/badge/Doosan--API-v1.0-success?logo=powerbi" />
  <img src="https://img.shields.io/badge/MoveIt2-Humble-informational?logo=autodesk" />
  <img src="https://img.shields.io/badge/RealSense-D435-0071C5?logo=intel" />
  <img src="https://img.shields.io/badge/LangChain-LLM-blueviolet?logo=openai" />
  <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg?logo=apache" />
</p>

# D-Cashier-
---
## 📌 System Overview

The **D-Cashier** project is a smart, voice-controlled automated checkout system designed for retail environments such as convenience stores or unmanned kiosks. It integrates object detection, voice interface, and robotic manipulation to streamline the checkout process.

This system enables users to interact entirely through voice, while products are recognized and processed using a YOLOv11n-OBB-based vision module. Unrecognized items are automatically handled via a “Cancel Position,” and restricted goods are verified through OCR and face recognition.

---

## 🎥 Demo

<p align="center">
  <a href="https://youtu.be/oQZS48vI508" target="_blank">
    <img src="https://img.youtube.com/vi/oQZS48vI508/0.jpg" alt="Demo Video" width="600"/>
  </a>
</p>

<p align="center">
  👉 Click the thumbnail above to watch the demo video on YouTube!
</p>

---

## 🔄 System Architecture

<p align="center">
  <img src="https://github.com/user-attachments/assets/d00baefa-ddab-47a7-8af0-e556145e6e24" width="38%" style="margin-right: 1%;" />
  <img src="https://github.com/user-attachments/assets/a1e296fb-d3d4-433d-b567-8fa027643bd4" width="58%" style="margin-left: 1%;" />
</p>


---

## 🔧 Core Achievements

- 🌀 **Multi-frame Object Detection + Rotation Estimation**  
  → Implemented a custom post-processing algorithm for YOLOv11n-OBB  
  → Achieved **±3° yaw error margin**

- ⏱ **Voice Interface with Real-Time GUI + TTS Feedback**  
  → System response time maintained under **1 second**

- ❌ **“Cancel Position” Handling for Undetected Items**  
  → Reduced false detection issues by over **40%**


---

### 🔍 Key Functional Features


- **① YOLOv11n-OBB Object Detection**  
  Detects objects and estimates their 3D position + orientation `[x, y, z, yaw]` using oriented bounding boxes.  
  Polygon vertices are averaged across frames to improve yaw estimation.  

  <p align="center">
    <img src="https://github.com/user-attachments/assets/4f12acba-6f9c-4920-88b6-de133854c6db" width="60%" />
  </p>

- **② Background Subtraction + Cancel Position Handling**  
  If YOLO fails to detect an object, the system compares the current frame with a pre-stored background image to locate unexpected items.  
  Detected unknown objects are moved to a **Cancel Position** to prevent false charges.

  <p align="center">
    <img src="https://github.com/user-attachments/assets/5e0055c8-21f1-42b7-8a52-5c9d880f92e5" width="60%" />
  </p>

- **③ Adult Verification (19+ Restricted Items)**  
  When a restricted item is detected (e.g., alcohol, cigarettes), the system:
  - Uses OCR to extract birth date from a captured ID card  
  - Matches the face from the ID with the user’s face in front of the camera  
  - Grants or denies approval based on age + match score

  <p align="center">
    <img src="https://github.com/user-attachments/assets/15c4903c-4b21-4fbe-8fc0-876b673caaaa" width="60%" />
  </p>

- **④ Voice-Controlled Interface with GUI Feedback**
  - Wake-up word detection: `"Hello Rokey"`  
  - Natural language input via OpenAI Whisper  
  - Intent parsing via **LangChain + GPT-4o**  
  - Real-time GUI update + TTS output using OpenAI voice
<p align="center">
  <a href="https://youtu.be/M5Lf-O-FUkQ" target="_blank">
    <img src="https://img.youtube.com/vi/M5Lf-O-FUkQ/0.jpg" alt="Demo Video" width="600"/>
  </a>
</p>

---

### 🤖 Vision-to-Robot Conversion + Manipulation

- **Pose Conversion**
  - Converts YOLO’s `[x, y, z, yaw]` to robot base coordinates `[x, y, z, rx, ry, rz]`
  - Uses `T_gripper2camera.npy` and external calibration parameters for accurate transform
  - Adjusts gripper width based on object size (e.g., `min_side × 10 - 50`)
  - Pick action is executed with Doosan’s `movel()` API

- **Cancel Preemption (Stop & Retry)**
  - User can say "정지" or "Remove [item]" → current goal is canceled
  - Robot switches to cancel pose using a **custom CancelObject service**
  - Uses `MultiThreadedExecutor` to handle cancel requests concurrently with execution

- **Force-Sensitive Grasping**
  - Grasp failure is detected when `|Fz|` force remains unchanged after closure
  - For fragile items (bottles, cans), **compliance control** is used:
    - Applies downward force (e.g., 15N @ Z-axis)
    - Releases when force drops below threshold (e.g., <10N)
  - Logs all force values and errors for safety validation

---

## 📄 Documentation

For a detailed explanation of this project, please refer to the following document:

👉 [docs](docs/F-1_협동2_한준모_배재성_김동호_김예신(중도포기).pdf)

---
## 👥 Contributors

Thanks to these wonderful people who have contributed to this project:

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/weedmo">
        <img src="https://github.com/weedmo.png" width="100px;" alt="weedmo"/><br />
        <sub><b>weedmo</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/jsbae-RL">
        <img src="https://github.com/jsbae-RL.png" width="100px;" alt="jsbae-RL"/><br />
        <sub><b>jsbae-RL</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/DONGHO1206">
        <img src="https://github.com/DONGHO1206.png" width="100px;" alt="DONGHO1206"/><br />
        <sub><b>DONGHO1206</b></sub>
      </a>
    </td>


</table>

