# 🛠️ CAD Design

This directory contains all the CAD models, 3D printable parts, and design files for the Ball Balancing Robot.

The mechanical design was created to provide a lightweight, rigid, and easily manufacturable platform capable of accurately tilting in two degrees of freedom (2-DOF). All components are designed for FDM 3D printing and can be assembled using standard hardware.

---

## 📂 Folder Structure

```
CAD/
├── STL/              # STL files for 3D printing
├── Fusion360/        # Fusion 360 source files
├── Images/           # CAD renders and assembly images
└── README.md
```

---

# 🧩 Components

The CAD model consists of the following major parts:

### 1. Base Plate
- Supports the complete robot assembly.
- Holds the servo mounting ring.
- Provides mounting points for electronics.

---

### 2. Servo Mounting Ring
- Securely holds all three servo motors.
- Ensures equal spacing between servos.
- Maintains platform symmetry.

---

### 3. Balancing Platform
- Circular top platform where the ball moves.
- Lightweight yet rigid design.
- Connected to the servo linkages for smooth motion.

---

### 4. Servo Arms
- Connect the servo horns to the balancing platform.
- Convert servo rotation into platform tilt.

---

### 5. Linkages
- Transfer motion from the servos to the platform.
- Designed to minimize backlash and friction.

---

## ⚙️ Design Specifications

| Parameter | Value |
|-----------|--------|
| Degrees of Freedom | 2 (Pitch & Roll) |
| Number of Servos | 3 |
| Printing Method | FDM 3D Printing |
| Material | PLA / PETG |
| Layer Height | 0.20 mm |
| Infill | 20–30% |
| Supports | Only where required |

---

# 🖨️ Recommended Print Settings

| Setting | Value |
|----------|-------|
| Layer Height | 0.20 mm |
| Nozzle Diameter | 0.4 mm |
| Wall Count | 3 |
| Top/Bottom Layers | 5 |
| Infill | 25% Gyroid |
| Material | PLA |
| Print Speed | 50–60 mm/s |

---

# 🔩 Hardware Required

- 3 × MG996R Servo Motors
- M3 Screws & Nuts
- Servo Horn Screws
- Ball Bearing (if applicable)
- External Servo Power Supply
- PCA9685 Servo Driver
- Raspberry Pi 4
- Raspberry Pi Camera

---

# 📐 Assembly

1. Print all STL files.
2. Install the servo motors into the mounting ring.
3. Fix the mounting ring onto the base plate.
4. Attach the servo horns.
5. Connect the linkages.
6. Mount the balancing platform.
7. Verify smooth movement before powering the servos.

---

# 📷 Assembly Preview

> Images and assembly renders will be added here.

```
Images/
├── Assembly.png
├── Exploded_View.png
├── Top_View.png
└── Side_View.png
```

---

# ⚠️ Notes

- Ensure all servo horns are mounted at their center (90°) position before assembly.
- Do not overtighten the linkage screws.
- Check for any mechanical interference before running the control software.
- Smooth and friction-free movement is essential for stable balancing performance.

---

# 🚀 Future Improvements

- Carbon fiber reinforcement for increased stiffness
- Adjustable linkage lengths
- Quick-release platform design
- Improved cable management
- Support for high-precision digital servos

---

## 📜 License

The CAD models are released under the same license as this repository.

If you modify or improve the design, contributions are welcome through pull requests.
