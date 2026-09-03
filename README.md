# hand-robot-control
# Hand Gesture Controlled Robotic System

**Created and Developed By:** Sanket Kshirsagar
**Role:** Senior Research Engineer
**Project:** Hand Gesture Controlled Robotic System

> **Description:**
> Main launcher for the hand-tracking and robotic control system. Provides real-time gesture-based robot operation, process management, monitoring, recalibration, and safe shutdown.

---

## 📌 Project Overview

This project enables real-time robotic control using hand gestures and computer vision. The system integrates hand tracking with robotic control to provide an intuitive human-machine interaction interface.

## ✨ Features

* Real-time hand gesture detection
* Gesture-based robotic arm control
* Gripper open/close control
* Emergency robot stop gesture
* Tracking-origin recalibration
* Real-time process management
* Robot speed adjustment
* Safe system shutdown
* Separate hand-tracking and robot-control modules

## 🎮 Gesture Controls

| Gesture                  | Action                      |
| ------------------------ | --------------------------- |
| ✊ Closed fist            | Recalibrate tracking origin |
| 🤏 Thumb + middle finger | STOP robot                  |
| ☝️ Thumb + index finger  | Open/close gripper          |

## ⌨️ Keyboard Controls

### Hand Tracking Window

* `Q` — Quit
* `C` — Manual recalibration

### Robot Control Window

* `ESC` — Quit
* `[` — Decrease speed
* `]` — Increase speed

## 📁 Project Structure

```text
hand-robot-control/
├── run_hand_robot.py
├── hand_tracker.py
├── robot_controller.py
└── README.md
```

## 🚀 Running the Project

Run the main launcher:

```bash
python run_hand_robot.py
```

## 👨‍💻 Developer

**Sanket Kshirsagar**
**Senior Research Engineer**

---

### Copyright

**Copyright © Sanket Kshirsagar. All Rights Reserved.**
