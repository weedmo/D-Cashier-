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
## 🛒 Overview

This project implements a **voice-controlled automated checkout system** for retail environments. Users can initiate and control the checkout process via speech, while the system identifies and processes items using computer vision.
---

### 🎯 Key Features

- ✅ **YOLO-Based Object Detection + Product Database Matching**  
  - Detects items in real-time using a camera and the YOLO model  
  - Matches detected items with a predefined JSON database to retrieve **product name, price, and category**

- 🚫 **Handling Undetected or Unknown Items (Cancel Position)**  
  - Items not detected by YOLO or not found in the database are automatically placed in a **"Cancel Position"**  
  - Prevents false charges and prompts user re-verification

- 🔞 **Adult Verification for Restricted Items**  
  - For age-restricted items (e.g., alcohol, cigarettes), the system verifies the user’s age by combining  
    **ID card OCR** and **face recognition matching**

- 🗣 **Voice-Based Interaction (Input & Feedback)**  
  - Users can issue natural language commands such as “Start checkout”, “Remove this”, or “Add this”  
  - Items can be **added or removed freely during the checkout process**

- 🖥️ **Real-Time GUI and Voice Feedback**  
  - The system provides real-time updates on actions (e.g., “Tofu added”, “Adult verification required”)  
  - Feedback is delivered **both visually via GUI and audibly via text-to-speech**

---

This system offers an **intuitive and flexible solution** for smart retail environments,  
suitable for integration into unmanned checkout counters, convenience stores, and automated kiosks.

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

