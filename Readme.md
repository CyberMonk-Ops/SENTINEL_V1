# 🚁 SENTINEL V1: Autonomous Pursuit System

> "The machine does not just observe. It hunts."

https://github.com/user-attachments/assets/8cef3257-f893-4a15-839d-98fed7523251

## 🛑 System Overview
The **SENTINEL V1** is an autonomous UAV defense unit built on **ROS 2 Jazzy**. Unlike standard patrol drones, it features a "Hostile Interrogation" logic layer that detects unauthorized personnel, locks onto their face coordinates, and executes a close-range intimidation maneuver.

## ⚡ Tech Stack
* **Core:** ROS 2 Jazzy (Python Nodes)
* **Vision:** OpenCV (Haar Cascade Face Tracking)
* **Simulation:** Gazebo Harmonic
* **Control:** Custom PID Loop for 3-Axis Stabilization

## 📂 Key Features
* **Finite State Machine (FSM):** Handles distinct behavioral modes (SEARCH -> TRACK -> INSPECT).
* **Dynamic HUD:** Real-time tactical display overlay with battery telemetry and warning systems.
* **Evidence Locker:** Automated logging system that captures high-res snapshots of intruders.

## 🚀 How to Run
1.  Clone the repository:
    ```bash
    git clone [https://github.com/CyberMonk-Ops/SENTINEL_V1.git](https://github.com/CyberMonk-Ops/SENTINEL_V1.git)
    ```
2.  Build the package:
    ```bash
    colcon build --packages-select my_bot_logic
    ```
3.  Launch the Hunter:
    ```bash
    ros2 run my_bot_logic machine_face
    ```

---
*Built by CyberMonk-Ops | 2026*




