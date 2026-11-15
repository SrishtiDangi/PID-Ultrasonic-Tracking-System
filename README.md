# PID Ultrasonic Tracking System

## Overview
This project uses an Arduino to balance/track an object at a specific distance using an Ultrasonic Sensor, a Servo Motor, and a PID controller. It also features an LCD display for live telemetry.

## Hardware Required
- 1x Arduino Uno / Nano / Mega
- 1x HC-SR04 Ultrasonic Sensor
- 1x SG90 or MG995 Micro Servo Motor
- 1x 16x2 LCD Display (Standard parallel interface, no I2C module used)
- Jumper wires & Breadboard

---

## 🔌 Wiring Guide

### 1. HC-SR04 Ultrasonic Sensor
| Sensor Pin | Arduino Pin |
|:---:|:---:|
| VCC | 5V |
| Trig | D9 |
| Echo | D10 |
| GND | GND |

### 2. Servo Motor
| Servo Wire | Arduino Pin |
|:---:|:---:|
| Red (Power) | 5V (Consider external power if servo twitches) |
| Brown/Black (GND)| GND |
| Yellow/Orange (Signal)| D6 |

### 3. 16x2 LCD Display
| LCD Pin | Function | Arduino Pin |
|:---:|:---|:---:|
| 1 (VSS) | Ground | GND |
| 2 (VDD) | Power (5V) | 5V |
| 3 (V0)  | Contrast | Middle leg of 10k Potentiometer |
| 4 (RS)  | Register Select | D2 |
| 5 (RW)  | Read/Write | GND |
| 6 (E)   | Enable | D3 |
| 11 (D4) | Data 4 | D4 |
| 12 (D5) | Data 5 | D5 |
| 13 (D6) | Data 6 | D7 |
| 14 (D7) | Data 7 | D8 |
| 15 (A)  | Backlight Anode | 5V (via 220Ω resistor) |
| 16 (K)  | Backlight Cathode | GND |

---

## 🛠️ Operating Instructions

### Option 1: Standalone Physics Simulator (No Hardware Needed!)
Perfect for presentations and software evaluations. Models second-order plant dynamics, acoustic noise, animated 2D graphics, and dynamic step response metrics ($t_r, t_s, M_p, e_{ss}$):
```bash
python simulator.py
```
- Interactive sliders for $K_p, K_i, K_d$, Target distance, and Sensor Noise.
- Click **"⚡ Inject Physical Disturbance (Nudge)"** to demonstrate real-time dynamic PID recovery.
- Can toggle between **Physics Simulation** and **Hardware USB** with a single click.

### Option 2: Live Hardware Telemetry GUI Dashboard
Connects to the physical Arduino over USB at 115200 baud for real-time oscilloscope tracking and live PID tuning:
```bash
python gui_monitor.py
```

### Option 3: Arduino Embedded Firmware
1. Open [`pid_control_main/pid_control_main.ino`](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/pid_control_main/pid_control_main.ino) in Arduino IDE and click **Upload**.
2. Open **Tools > Serial Plotter** (`Ctrl + Shift + L`) at **115200 baud** to view real-time tracking curves.
3. LCD displays real-time telemetry ($T, D, E, S$) and failsafe status.

---

## 📚 Academic Documentation
- [**Full 100-Mark Project Report**](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/Docs/Full_Project_Report.md): Complete mathematical modeling ($s$-domain & $z$-domain), noise mitigation derivations, transient stability analysis ($t_r, t_s, M_p$), and experimental results.
- [**Viva & Presentation Cheatsheet**](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/Docs/Project_Nodes.md): 30-second presentation elevator pitch, block diagrams, and answers to the top professor questions.
