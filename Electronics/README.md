# ⚡ Electronics

This folder contains the electronic architecture, wiring information, and hardware documentation for the **Ball Balancing Robot**.

The current system uses a **Raspberry Pi 4** as the main controller, a **Raspberry Pi Camera** for visual feedback, a **PCA9685 16-channel PWM servo driver** for servo control, and **three servo motors** for positioning the balancing platform.

A dedicated **5V power supply** is used for the servo system.

---

## 🧠 System Architecture

```text
                    ┌─────────────────────┐
                    │    Raspberry Pi 4   │
                    │   Main Controller   │
                    └──────────┬──────────┘
                               │
                 ┌─────────────┴─────────────┐
                 │                           │
                 ▼                           ▼
        ┌─────────────────┐        ┌─────────────────┐
        │ Raspberry Pi    │        │    PCA9685      │
        │     Camera      │        │  PWM Driver     │
        └────────┬────────┘        └────────┬────────┘
                 │                          │
                 │                    ┌─────┼─────┐
                 │                    │     │     │
                 │                    ▼     ▼     ▼
                 │                  Servo Servo Servo
                 │                    1     2     3
                 │
                 ▼
        Ball Position Detection
                 │
                 ▼
        Control Algorithm
                 │
                 ▼
        Servo Commands
                 │
                 ▼
        Platform Adjustment
```

---

# 🔧 Main Hardware

| Component | Quantity | Function |
|-----------|----------|----------|
| Raspberry Pi 4 | 1 | Main controller and image processing |
| Raspberry Pi Camera | 1 | Visual detection of the ball |
| PCA9685 | 1 | PWM servo controller |
| Servo Motors | 3 | Positioning and adjustment of the balancing platform |
| 5V Power Supply | 1 | External power for the servo system |

---

# 🖥️ Raspberry Pi 4

The **Raspberry Pi 4** acts as the main processing and control unit of the robot.

Its primary responsibilities include:

- Camera interfacing
- Image processing
- Ball detection
- Ball position estimation
- Control algorithm execution
- Generating servo commands
- Communicating with the PCA9685 through I²C

The Raspberry Pi provides the computational platform required for implementing the vision-based balancing system.

---

# 🎥 Raspberry Pi Camera

The Raspberry Pi Camera provides the visual feedback required to determine the position of the ball on the balancing platform.

The camera is connected to the Raspberry Pi 4 and is used for image acquisition.

The intended processing pipeline is:

```text
Camera
   ↓
Image Capture
   ↓
Image Processing
   ↓
Ball Detection
   ↓
Ball Position
   ↓
Position Error
   ↓
Control Algorithm
   ↓
Servo Commands
```

The camera-based feedback system is intended to enable closed-loop control of the balancing platform.

---

# 🎛️ PCA9685 Servo Driver

The **PCA9685 16-channel PWM servo driver** is used to control the three servo motors.

The Raspberry Pi 4 communicates with the PCA9685 through the **I²C interface**.

```text
Raspberry Pi 4
      │
      │ I²C
      ▼
   PCA9685
      │
 ┌────┼────┐
 │    │    │
 ▼    ▼    ▼
S1    S2   S3
```

Using the PCA9685 provides dedicated PWM generation for the servos while allowing the Raspberry Pi 4 to focus on vision processing and control calculations.

---

# ⚙️ Servo Configuration

Three servo motors are currently used in the balancing mechanism.

| PCA9685 Channel | Device |
|-----------------|--------|
| Channel 0 | Servo 1 |
| Channel 1 | Servo 2 |
| Channel 2 | Servo 3 |

The channel assignments can be modified in software if required.

The exact mechanical role of each servo depends on the current balancing-platform design.

---

# 🔌 Raspberry Pi 4 ↔ PCA9685

The PCA9685 communicates with the Raspberry Pi 4 using the I²C interface.

Typical connections are:

| Raspberry Pi 4 | PCA9685 |
|----------------|---------|
| 3.3V | Logic VCC* |
| GND | GND |
| SDA | SDA |
| SCL | SCL |

The servo motors receive power through the PCA9685's external servo power input.

> **Important:** The exact power connections should follow the requirements of the specific PCA9685 board being used.

---

# 🔋 Power System

A dedicated **5V power supply** is used for the servo system.

The servo motors should not be powered directly from the Raspberry Pi 4's GPIO pins.

The basic power architecture is:

```text
                 5V Power Supply
                       │
                       ▼
                PCA9685 Servo Rail
                       │
              ┌────────┼────────┐
              │        │        │
              ▼        ▼        ▼
           Servo 1  Servo 2  Servo 3


                Raspberry Pi 4
                       │
                       │ I²C
                       ▼
                    PCA9685
```

The Raspberry Pi and servo system should have a **common ground** so that the PWM control signals have a common electrical reference.

> ⚠️ Servo motors can draw significant current, especially during acceleration or when mechanically loaded. The 5V power supply should therefore be appropriately rated for the combined servo load.

---

# 🔄 Control Loop

The intended balancing control loop is:

```text
        ┌──────────────────────┐
        │   Raspberry Pi       │
        │      Camera          │
        └──────────┬───────────┘
                   │
                   ▼
            Capture Image
                   │
                   ▼
            Detect Ball
                   │
                   ▼
          Calculate Position
                   │
                   ▼
           Calculate Error
                   │
                   ▼
          Control Algorithm
                   │
                   ▼
             Servo Angles
                   │
                   ▼
              PCA9685
                   │
                   ▼
             3 Servos
                   │
                   ▼
          Platform Movement
                   │
                   └───────────────┐
                                   │
                                   ▼
                             Ball Movement
                                   │
                                   └──→ Camera
```

This creates the basis for a closed-loop visual balancing system.

---

# 🧩 Electronic Design Goals

The electronics are designed around the following goals:

- Raspberry Pi-based processing
- Camera-based visual feedback
- Dedicated PWM servo control
- External servo power
- Modular wiring
- Simple debugging
- Expandability
- Closed-loop control development

---

# 🧪 Development Status

| Subsystem | Status |
|-----------|--------|
| Raspberry Pi 4 | ✅ Available |
| Raspberry Pi Camera | ✅ Available |
| PCA9685 | ✅ Available |
| Three Servo Motors | ✅ Available |
| 5V Power Supply | ✅ Available |
| Raspberry Pi ↔ PCA9685 Communication | 🚧 Development |
| Servo Control | 🚧 Development |
| Camera Processing | 🚧 Development |
| Ball Detection | 🚧 Development |
| Closed-Loop Balancing | ⏳ Planned |

---

# 🚀 Future Development

The electronics and control system will be progressively developed toward:

- Camera calibration
- Real-time ball detection
- Ball position tracking
- Servo calibration
- Closed-loop feedback
- PID-based balancing
- Platform position control
- Improved control response
- Automatic center-position calibration
- Stable ball balancing

---

# 📐 Wiring Documentation

The circuit and wiring diagrams for the robot will be maintained in this folder.

The documentation will be updated as the hardware architecture evolves.

```text
Electronics/
│
├── README.md
├── Circuit_Diagram.*
└── Wiring_Diagram.*
```

Replace the example filenames above with the actual diagram filenames present in the repository.

---

# 📁 Project Structure

```text
Ball-Balancing-Robot/
│
├── CAD/
│
├── Electronics/
│   ├── README.md
│   ├── Circuit_Diagram.*
│   └── Wiring_Diagram.*
│
├── Code/
│
├── Images/
│
├── Documentation/
│
└── README.md
```

---

# 📌 Notes

This electronics architecture represents the current prototype.

The hardware configuration may change as the mechanical design, servo control, computer vision, and balancing algorithm are further developed.

All future hardware revisions and wiring changes will be documented in this folder.
