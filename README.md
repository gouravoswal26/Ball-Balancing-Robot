# 🎯 Ball Balancing Robot

A 2-DOF vision-based Ball Balancing Robot that uses computer vision, inverse kinematics, and PID control to keep a ball balanced at the center of a tilting platform. The system is powered by a Raspberry Pi 4, Raspberry Pi Camera, PCA9685 Servo Driver, and MG996R servo motors.

<p align="center">
  <img src="images/robot.jpg" width="600">
</p>

---

## 📌 Project Overview

The Ball Balancing Robot is a closed-loop control system designed to stabilize a rolling ball on a circular platform. A Raspberry Pi Camera continuously tracks the ball's position using OpenCV. The positional error is processed through a PID controller, and the calculated correction is converted into servo angles using inverse kinematics. The platform is then tilted accordingly to bring the ball back to the desired position.

This project combines concepts from:

- Robotics
- Computer Vision
- Embedded Systems
- Control Systems
- Inverse Kinematics
- Mechanical Design

---

## ✨ Features

- 🎥 Real-time ball tracking using OpenCV
- 🎯 Closed-loop PID control
- ⚙️ 3-Servo 2-DOF balancing platform
- 📷 Raspberry Pi Camera integration
- 🔄 Real-time servo angle calculation
- 🖨️ Fully 3D printable mechanical structure
- 📊 Live visualization of ball position
- 📈 Easy PID parameter tuning

---

## 🛠 Hardware Used

| Component | Quantity |
|-----------|----------|
| Raspberry Pi 4 | 1 |
| Raspberry Pi Camera Module | 1 |
| MG996R Servo Motor | 3 |
| PCA9685 Servo Driver | 1 |
| 5V External Power Supply | 1 |
| Ball Balancing Platform (3D Printed) | 1 |
| Steel Ball | 1 |

---

## 💻 Software Stack

- Python 3
- OpenCV
- NumPy
- Adafruit PCA9685 Library
- Raspberry Pi OS

---

## 🧠 Working Principle

1. The Raspberry Pi Camera captures live video.
2. OpenCV detects the position of the ball.
3. The error between the current and desired position is calculated.
4. A PID controller computes the correction.
5. Inverse kinematics converts the correction into servo angles.
6. The PCA9685 drives the servos.
7. The platform tilts, moving the ball back to the center.

---

## 📂 Project Structure

```
Ball-Balancing-Robot/
│
├── CAD/
│   ├── STL Files
│   └── Fusion360 Files
│
├── Electronics/
│   ├── Circuit Diagram
│   └── Wiring
│
├── Software/
│   ├── Vision
│   ├── PID Controller
│   ├── Servo Control
│   └── Main Program
│
├── Images/
├── Videos/
├── README.md
└── LICENSE
```

---

## 🔧 System Architecture

```
             Raspberry Pi Camera
                     │
                     ▼
                OpenCV Vision
                     │
                     ▼
              Ball Coordinates
                     │
                     ▼
              PID Controller
                     │
                     ▼
           Inverse Kinematics
                     │
                     ▼
             PCA9685 Servo Driver
                     │
                     ▼
             MG996R Servo Motors
                     │
                     ▼
            Ball Balancing Platform
```

---

## 📷 Mechanical Design

The balancing platform is fully designed in CAD and optimized for 3D printing. It consists of:

- Circular balancing plate
- Servo mounting ring
- Linkage mechanism
- Base plate
- Adjustable servo arms

---

## 📸 Demo

Coming Soon

---

## 📈 Future Improvements

- Adaptive PID
- Kalman Filter
- Machine Learning Based Controller
- Faster Digital Servos
- ESP32/STM32 Real-Time Controller
- Web Dashboard for PID Tuning
- Automatic Camera Calibration

---

## 📚 Learning Outcomes

This project helped me understand:

- Closed-loop feedback control
- PID tuning
- Inverse kinematics
- Raspberry Pi programming
- Computer vision using OpenCV
- Mechanical design for robotics
- System integration

---

## 📄 License

This project is released under the MIT License.

---

## 👨‍💻 Author

**Gourav Jain**

Electronics & Communication Engineering  
Thapar Institute of Engineering & Technology

GitHub: https://github.com/gouravoswal26
