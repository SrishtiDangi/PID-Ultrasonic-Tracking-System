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
1. Upload `pid_control_main.ino` to your Arduino.
2. Open the **Serial Monitor** and set the baud rate to **115200**.
3. Place an object in front of the sensor. The servo will actuate to track it to the target distance (default 20cm).

**Live Tuning via Serial Monitor:**
You can update PID values without recompiling! Type the following into the Serial Monitor:
- `P2.5` -> Sets Kp to 2.5
- `I0.05` -> Sets Ki to 0.05
- `D0.15` -> Sets Kd to 0.15
- `T15` -> Sets Target Distance to 15 cm
