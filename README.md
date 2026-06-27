# AI-Powered Robotic Sorting System

An autonomous robot system that uses computer vision and deep learning to detect, pick up, and sort objects (aluminum cans and cartons) based on their visual categories. Designed for educational robotics competitions and industrial automation demonstrations.

---

## 🧠 Project Overview

This project implements a complete **end-to-end robotic perception and control pipeline**:

- **Object Detection** – Fine‑tuned YOLOv8 model trained to recognise **aluminum cans**, **cartons**, and **coloured indicator balls** (red/green).
- **Autonomous Navigation** – State‑machine based motion planning that drives a differential‑drive robot to approach, grasp, and transport objects.
- **Intelligent Sorting** – Objects are dropped at designated zones based on their class (aluminum vs. carton), with a mission completion condition (2 of each).
- **Real‑time Feedback** – Live video overlay with bounding boxes, class labels, width/distance indicators, and current state/counter displays.

The system is written in **Python** and runs on a Linux/Windows PC connected to a robotic platform via **USB‑to‑Serial** (UART) communication.

---

## 🚀 Key Features

| Feature | Description |
|---------|-------------|
| **Deep Learning Detection** | YOLOv8 custom model (`best.pt`) with confidence threshold 0.2, running at ~15‑20 FPS on CPU/GPU. |
| **Dynamic State Machine** | Clear phases: `INIT_FORWARD` → `SEARCH_ITEM` → `POST_GRAB_BACKWARD` → `MOVE_TO_ZONE` → `POST_DROP_BACKWARD` → `MISSION_DONE`. |
| **Object Classification** | Distinguishes between `mini-cola-cans`, `sprite`, `vitasoy-soyabean-milk`, `lemon-tea` (and generic aliases). |
| **Ball‑Guided Drop Zones** | Two coloured balls (green = aluminum zone, red = carton zone) act as visual landmarks for precise dropping. |
| **Adaptive Scanning** | If no target is visible, the robot executes a pre‑programmed scan routine (turn left 1s, hold, turn right 2s, hold) until the target appears. |
| **Gripper Control** | Servo‑based gripper opens/closes via PWM commands over serial. |
| **Visual Debug Overlay** | Annotated camera feed shows bounding boxes, center coordinates, object width, distance label (Near/Far), current state, and item counters. |

---

## 🛠️ Technology Stack

- **Programming Language**: Python 3.8+
- **Machine Learning**: [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) (custom model)
- **Computer Vision**: OpenCV (`cv2`)
- **Serial Communication**: PySerial
- **Robot Control**: Custom `lib3360` library (sends PWM/servo commands via UART)
- **State Management**: Time‑based finite‑state machine (FSM)

---

## 📦 Hardware Setup

- **Robot Platform**: Differential‑drive mobile robot with two motors and a servo‑actuated gripper.
- **Camera**: USB webcam (default `0`) with resolution 800×600.
- **Microcontroller**: STM32 (or compatible) receiving UART packets at 115200 baud.
- **Serial Port**: Configurable via environment variable `SENDSERIAL_PORT` or command‑line argument.
- **Power**: Ensure stable power to motors/servos and logic board.

> **Note**: The serial protocol uses a fixed 11‑byte frame: header `0x0D` + 9‑byte payload (4× uint16 PWM + 1× direction byte) + footer `0x20`. See `transmitter.py` for details.

---

## 🧩 File Structure

```
├── Cans-Cartons-Classification.py   # Main script for sorting aluminum vs. carton (2 each)
├── Cola-Classification.py           # Alternative mission: collect only cola cans (3 total)
├── lib3360.py                       # Library wrapper for motor/servo control over serial
├── transmitter.py                   # Low‑level serial frame builder and sender
├── receiver.py                      # Example serial receiver (for debugging)
├── 3360lib.py                       # Compatibility loader for importlib
├── example.py                       # Demo usage of the library
├── README.md                        # This file
└── Library_Robot/                   # (optional) legacy folder for lib3360
    └── lib3360.py
```

> **Training Data & Model**: The custom YOLO model `best.pt` is stored separately (not included here) and should be placed at `./IC_AI_Project_new/IC_AI_Model_06/result/train/weights/best.pt`. Adjust the path in the scripts if needed.

---

## ⚙️ Installation & Setup

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/robotic-sorting-system.git
cd robotic-sorting-system
```

### 2. Create a Virtual Environment (recommended)
```bash
python -m venv venv
source venv/bin/activate      # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```
If no `requirements.txt` exists, install manually:
```bash
pip install ultralytics opencv-python pyserial
```

### 4. Download the YOLO Model
Place your trained `best.pt` file in the expected directory, or update the path in the scripts (`m_Model_01 = YOLO("./IC_AI_Project_new/...")`).

### 5. Configure Serial Port
- **Linux** (typical): `/dev/ttyUSB0` or `/dev/ttyTHS1` (Jetson)
- **Windows**: `COM15` (example)
Override via:
```bash
export SENDSERIAL_PORT=/dev/ttyUSB0   # Linux/macOS
set SENDSERIAL_PORT=COM15             # Windows cmd
```
Or pass as command‑line argument: `python Cans-Cartons-Classification.py COM15`.

---

## 🚦 How to Run

### Sort Cans vs. Cartons (2 each)
```bash
python Cans-Cartons-Classification.py
```

### Collect Only Cola Cans (3 total)
```bash
python Cola-Classification.py
```

Press `q` at any time to safely stop the robot and release the camera.

---

## 🔄 State Machine (Cans‑Cartons)

| State | Action |
|-------|--------|
| `INIT_FORWARD` | Move forward 3s to clear starting position. |
| `SEARCH_ITEM` | Detect the next target category (aluminum first, then carton) and align/approach. |
| `POST_GRAB_BACKWARD` | Reverse 7s after grasping. |
| `POST_GRAB_TURN` | Turn toward the correct drop zone (left for aluminum, right for carton). |
| `MOVE_TO_ZONE` | Locate the coloured ball (green/red) and position the robot to drop. |
| `POST_DROP_BACKWARD` | Reverse 5s after dropping. |
| `POST_DROP_TURN_*` | Turn back to search for next item. |
| `MISSION_DONE` | Reverse, turn, open gripper, and stop. |

---

## 🧪 Demonstration

Below is a sample output from the live video feed (annotated):

```
STATE: SEARCH_ITEM | AL: 0 | CA: 0
[bounding boxes with class names, center coords, width, "Near"/"Far"]
```

**Expected Behaviour**:
1. Robot moves forward, then begins scanning.
2. Detects an aluminum can (e.g., `mini-cola-cans`) → aligns, moves close, closes gripper.
3. Reverses, turns left (toward green ball), finds green ball, drops the can to the left of it.
4. Repeats for second aluminum can (drops to the right of green ball).
5. Switches to carton detection, repeats similar steps using the red ball.
6. After collecting 2 aluminum + 2 carton, performs finishing manoeuvre and stops.

---

## 📈 Performance & Tuning

- **Detection Confidence**: Set to `0.2` in the scripts; adjust if false positives/negatives occur.
- **Motor PWM Values**: Modify `FWD_PWM_L/R`, `CURVE_FAST/SLOW`, etc., to match your robot’s speed/torque.
- **Thresholds**: `DROP_NEAR_WIDTH`, `PICK_NEAR_WIDTH`, `CAM_TOL`, and `desired_x` positions can be calibrated for your specific arena layout.
- **Scan Timing**: The scan routine (`TURN_L`, `HOLD_L`, `TURN_R`, `HOLD_R`) may need adjustment based on field of view and arena size.
