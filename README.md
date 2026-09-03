# 🤖 Hand Gesture Controlled Robotic System

**Created and Developed By:** Sanket Kshirsagar
**Role:** Senior Research Engineer
**Project:** Hand Gesture Controlled Robotic System

> **Description:**
> A real-time hand gesture-based robotic control system integrating computer vision, hand tracking, process management, robotic motion control, gripper operation, recalibration, monitoring, and safe shutdown.

---

## 📌 Overview

The **Hand Gesture Controlled Robotic System** is a human-machine interaction platform designed to control a robotic system using real-time hand gestures.

The system separates **hand-tracking** and **robot-control** functionality into dedicated processes while providing a unified launcher for coordinated execution and safe shutdown.

The architecture is designed for real-time operation and provides gesture-based control, recalibration, gripper operation, robot stop functionality, speed adjustment, and process monitoring.

---

## ✨ Key Features

* 🖐️ Real-time hand gesture recognition
* 🤖 Gesture-based robotic control
* ✋ Hand-tracking based human-machine interaction
* 🦾 Robotic motion control
* 🔧 Gripper open/close control
* 🛑 Dedicated emergency STOP gesture
* 🎯 Tracking-origin recalibration
* ⚡ Real-time robot speed adjustment
* 🔄 Independent hand-tracking and robot-control processes
* 🖥️ Process monitoring and management
* 🛡️ Controlled process termination
* 🔌 Safe shutdown handling
* ⌨️ Keyboard-based manual controls

---

# 🏗️ System Architecture

The system follows a modular architecture consisting of a launcher, hand-tracking module, and robotic control module.

```text
                 ┌─────────────────────────┐
                 │     User Hand Input     │
                 │    Hand / Gestures      │
                 └────────────┬────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
                 │     Hand Tracker        │
                 │   hand_tracker.py       │
                 │                         │
                 │ Gesture Detection       │
                 │ Tracking                │
                 │ Recalibration           │
                 └────────────┬────────────┘
                              │
                              │ Gesture / Tracking Data
                              ▼
                 ┌─────────────────────────┐
                 │   Robot Controller      │
                 │  robot_controller.py    │
                 │                         │
                 │ Motion Control          │
                 │ Gripper Control         │
                 │ STOP                    │
                 │ Speed Control           │
                 └────────────┬────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
                 │    Robotic System       │
                 │                         │
                 │ Robot + End Effector    │
                 └─────────────────────────┘


                 ┌─────────────────────────┐
                 │   Main Process Manager  │
                 │  run_hand_robot.py      │
                 │                         │
                 │ Start / Stop Processes  │
                 │ Signal Handling         │
                 │ Process Monitoring      │
                 │ Safe Shutdown           │
                 └────────────┬────────────┘
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
             Hand Tracker          Robot Controller
```

---

# 🧩 Software Architecture

The project is divided into three primary Python modules.

### `run_hand_robot.py`

Main application launcher and process manager.

Responsibilities include:

* Starting the hand-tracking process
* Starting the robot-control process
* Coordinating both processes
* Handling termination signals
* Managing safe shutdown
* Displaying available control commands

### `hand_tracker.py`

Responsible for the hand-tracking and gesture-recognition side of the system.

Responsibilities include:

* Capturing hand movement
* Detecting predefined gestures
* Tracking hand position
* Recalibrating the tracking origin
* Providing gesture information to the control system

### `robot_controller.py`

Responsible for robotic control.

Responsibilities include:

* Processing control input
* Controlling robot movement
* Controlling the gripper
* Handling the STOP command
* Adjusting robot speed
* Managing robot-control execution

---

# 🎮 Gesture Controls

| Gesture                  | Function                    |
| ------------------------ | --------------------------- |
| ✊ Closed Fist            | Recalibrate tracking origin |
| 🤏 Thumb + Middle Finger | STOP robot                  |
| ☝️ Thumb + Index Finger  | Open / Close gripper        |

> Gesture definitions are based on the current project control logic.

---

# ⌨️ Keyboard Controls

## Hand Tracking Window

| Key | Function             |
| --- | -------------------- |
| `Q` | Quit                 |
| `C` | Manual recalibration |

## Robot Control Window

| Key   | Function             |
| ----- | -------------------- |
| `ESC` | Quit                 |
| `[`   | Decrease robot speed |
| `]`   | Increase robot speed |

---

# 🔄 System Workflow

```text
1. Start application
        │
        ▼
2. Launch Hand Tracker
        │
        ▼
3. Initialize gesture tracking
        │
        ▼
4. Launch Robot Controller
        │
        ▼
5. Detect hand gestures
        │
        ▼
6. Translate gestures into control commands
        │
        ▼
7. Execute robotic operation
        │
        ├───────────────┐
        │               │
        ▼               ▼
   Gripper Control   Robot Motion
        │               │
        └───────┬───────┘
                ▼
        Continuous Operation
                │
                ▼
        STOP / Quit Command
                │
                ▼
        Safe Process Shutdown
```

---

# 🛠️ Technology Stack

### Programming

* **Python**
* Object-oriented and modular programming
* Multiprocessing / subprocess-based process management
* Signal handling

### Computer Vision

* Real-time camera processing
* Hand tracking
* Gesture recognition

### Robotics

* Robotic arm control
* End-effector / gripper control
* Real-time motion commands

### System Integration

* Inter-process execution
* Process monitoring
* Safe termination
* Hardware-software integration

---

# 🔧 Hardware Requirements

The exact hardware configuration should match the robotic system used for deployment.

Typical system components include:

* 🤖 Robotic arm
* 🖐️ Camera / vision sensor
* 🔧 Robotic gripper / end effector
* 💻 Computer capable of running Python and real-time vision processing
* 🔌 Required robot communication interface
* ⚡ Appropriate power and control infrastructure

> **Note:** Exact hardware models, communication interfaces, and electrical specifications should be documented according to the deployed robotic setup.

---

# 💻 Software Requirements

Recommended environment:

* Python 3.x
* Windows / compatible operating environment
* Required Python packages listed in `requirements.txt`

Install dependencies using:

```bash
pip install -r requirements.txt
```

---

# 🚀 Installation

## 1. Clone the Repository

```bash
git clone https://github.com/sanket-dev-io/hand-robot-control.git
```

## 2. Enter the Project Directory

```bash
cd hand-robot-control
```

## 3. Install Dependencies

```bash
pip install -r requirements.txt
```

## 4. Connect Required Hardware

Connect and initialize the required camera, robotic system, and communication interfaces.

## 5. Run the Application

```bash
python run_hand_robot.py
```

---

# 📁 Project Structure

```text
hand-robot-control/
│
├── run_hand_robot.py
│   └── Main launcher and process manager
│
├── hand_tracker.py
│   └── Hand tracking and gesture recognition
│
├── robot_controller.py
│   └── Robotic control and gripper operation
│
├── requirements.txt
│   └── Python dependencies
│
└── README.md
    └── Project documentation
```

---

# 🛡️ Safety Considerations

This system controls physical robotic equipment. Appropriate safety precautions must be followed during operation.

The software provides control mechanisms including:

* Dedicated robot STOP gesture
* Manual termination controls
* Process termination handling
* Controlled shutdown of running processes
* Tracking-origin recalibration

Always ensure the robot operating area is safe before enabling motion.

---

# 🧪 Development & Research

The project is intended as a platform for research and development in:

* Human-Robot Interaction (HRI)
* Gesture-Based Robot Control
* Computer Vision
* Real-Time Hand Tracking
* Robotic Automation
* Intelligent Human-Machine Interfaces
* Vision-Based Robotic Systems
* Embedded and Industrial Automation

---

# 📸 Screenshots & Demonstrations

Screenshots, system architecture images, videos, and experimental results can be added here as the project documentation develops.

Example:

```text
docs/
├── images/
│   ├── system_setup.png
│   ├── hand_tracking.png
│   └── robot_control.png
│
└── videos/
    └── demonstration.mp4
```

---

# 👨‍💻 Author

**Sanket Kshirsagar**
**Senior Research Engineer**

Created and developed as an engineering/research project focused on intelligent human-robot interaction and robotic automation.

---

# 📜 Copyright

**Copyright © Sanket Kshirsagar. All Rights Reserved.**

Unauthorized reproduction, modification, or redistribution of this project or its source code is not permitted without appropriate authorization.

---

## ⭐ Project

**Hand Gesture Controlled Robotic System**

> Real-time computer vision + gesture recognition + robotic control + process management


🤝 Support & Contributions

For robot-side code, robotic integration, technical assistance, or project-related queries, please feel free to reach out.

📧 Email: infome.sanket@gmail.com

You can also use the GitHub Issues / Comments section to report bugs, suggest improvements, or ask technical questions related to the project.

If you find an issue or have a suggestion that can improve the robotic control system, your feedback is welcome.
