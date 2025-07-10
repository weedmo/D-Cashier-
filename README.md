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

## 🔧 Core Achievements

- 🌀 **Multi-frame Object Detection + Rotation Estimation**  
  → Implemented a custom post-processing algorithm for YOLOv11n-OBB  
  → Achieved **±3° yaw error margin**

- ⏱ **Voice Interface with Real-Time GUI + TTS Feedback**  
  → System response time maintained under **1 second**

- ❌ **“Cancel Position” Handling for Undetected Items**  
  → Reduced false detection issues by over **40%**

---

## 🧠 System Architecture

- **YOLOv11n-OBB Detection Node**
  - Detects items and estimates rotation (yaw)
  - Performs multi-frame averaging and post-processing
  - Outputs object pose for robot manipulation

- **Speech Interface Node**
  - Wake-up detection ("Hello Rokey")
  - Converts speech to text using OpenAI Whisper
  - Intent classification via GPT-4o (LangChain)

- **GUI + Voice Feedback**
  - Dynamic UI updates (cart, total price, alerts)
  - Text-to-speech voice announcements (OpenAI TTS)

- **Robot Control Node**
  - Pose conversion (camera to robot base)
  - Pick & Place execution via Doosan API
  - Compliant grasping with force sensors (RG2 gripper)

- **Cancel & Safety Handling**
  - Allows command-based canceling of current goal
  - Lookup and move to cancel position safely
  - Uses multithreading to handle interruption requests

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

