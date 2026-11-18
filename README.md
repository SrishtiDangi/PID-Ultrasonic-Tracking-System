# Precision PID Ultrasonic Tracking & Ball-on-Beam Control System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python: 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![Arduino: Uno/Nano](https://img.shields.io/badge/Hardware-Arduino_Uno-teal.svg)](https://www.arduino.cc/)
[![Wokwi Simulation](https://img.shields.io/badge/Wokwi-Virtual_Silicon_Proof-orange.svg)](https://wokwi.com/projects/new/arduino-uno)

A comprehensive final-year engineering capstone project implementing precision closed-loop control of an inherently open-loop unstable **ball-on-beam dynamic plant** ($G(s) = \frac{1.747}{s^2}$) using an **HC-SR04 ultrasonic transducer**, **SG90 micro-servo actuator**, **Arduino Uno (ATmega328P)**, and **real-time Python/Web-Serial telemetric dashboards**.

---

## 📑 Table of Contents
1. [System Overview & Control Architecture](#-system-overview--control-architecture)
2. [Physical Units & System Constants](#-physical-units--system-constants)
3. [Bill of Materials (BOM) & Hardware Specs](#-bill-of-materials-bom--hardware-specs)
4. [Hardware Wiring Diagram](#-hardware-wiring-diagram)
5. [Modes of Operation & How to Run](#-modes-of-operation--how-to-run)
6. [Wokwi Virtual Hardware Simulation (Honest Disclaimer)](#-wokwi-virtual-hardware-simulation)
7. [System Identification & Hardware Validation](#-system-identification--hardware-validation)
8. [Comprehensive Viva Examination Guide](#-comprehensive-viva-examination-guide)

---

## 🎯 System Overview & Control Architecture

The ball-on-beam rig is an archetype of non-linear mechatronics and control theory. Because the beam angle directly governs the linear acceleration of the ball ($\ddot{x} \propto \sin\alpha \approx \alpha$), the plant is a **pure double integrator**:
$$G_p(s) = \frac{X(s)}{\Theta(s)} = \frac{K_{\text{plant}}}{s^2}, \quad K_{\text{plant}} = \frac{5}{7} g K_{\text{linkage}} \approx 1.747\text{ cm/s}^2/\text{deg}$$

### Mathematical Highlights
* **Derivative on Measurement:** The controller evaluates $u_D(t) = -K_d \frac{dy}{dt}$ rather than $\frac{de}{dt}$, completely preventing "derivative kick" when the setpoint steps.
* **Conditional Anti-Windup:** Integral accumulation is clamped to $[-25^\circ, +25^\circ]$ and frozen during actuator saturation $[25^\circ, 155^\circ]$.
* **Deterministic Timing:** Firmware runs on a fixed $33.3\text{ Hz}$ ($30\text{ ms}$) `micros()` scheduler with zero-allocation buffers.
* **Acoustic Noise Mitigation:** Ultrasonic echoes are filtered via selectable Exponential Moving Average (EMA, $\alpha=0.30$) or a 1D Discrete Kalman Filter ($Q=0.04, R=0.35$).

```
[ Setpoint r(t) ] ---> (+) ---> [ Error e(t) ] ---> [ PID Controller ] ---> [ Servo Slew Limiter ] ---> [ SG90 Servo ]
                        ^ -                              |                                              |
                        |                                |                                              v
                        |                         [ Anti-Windup ]                               [ Beam Tilt Angle ]
                        |                                                                               |
                        +--- [ Kalman / EMA Filter ] <--- [ HC-SR04 Echo ] <--- [ Ball Position x(t) ] <-+
```

---

## 📐 Physical Units & System Constants

To ensure zero ambiguity during viva evaluations, all physical units and constants are strictly adhered to across firmware, simulator, and analysis suites:

| Parameter | Symbol | Nominal Value | Engineering Units |
| :--- | :--- | :--- | :--- |
| Beam Working Length | $L$ | $40.0$ | $\text{cm}$ |
| Gravitational Acceleration | $g$ | $981.0$ | $\text{cm/s}^2$ |
| Solid Sphere Rolling Factor | $c_{\text{roll}} = \frac{5}{7}$ | $0.7143$ | Dimensionless |
| Servo Horn Arm Radius | $r_{\text{arm}}$ | $2.0$ | $\text{cm}$ |
| Beam Pivot-to-Linkage Pitch | $L_{\text{linkage}}$ | $14.0$ | $\text{cm}$ |
| Effective Linkage Ratio | $K_{\text{linkage}}$ | $0.1429$ | $\text{deg/deg}$ |
| Linearized Plant Open-Loop Gain | $K_{\text{plant}}$ | $1.747$ | $\text{cm/s}^2/\text{deg}$ |
| Servo Neutral Level Position | $\theta_{\text{rest}}$ | $90.0$ | $\text{deg}$ |
| Servo Physical Travel Range | $[\theta_{\text{min}}, \theta_{\text{max}}]$ | $[25.0, 155.0]$ | $\text{deg}$ |
| Servo Maximum Slew Rate | $\dot{\theta}_{\text{max}}$ | $250.0$ | $\text{deg/s}$ |
| Ultrasonic Sample Loop Time | $\Delta t$ | $0.030$ ($30\text{ ms}$) | $\text{s}$ ($33.3\text{ Hz}$) |
| Ultrasonic Transducer Timeout | $t_{\text{timeout}}$ | $5000$ | $\mu\text{s}$ |
| Proportional Gain | $K_p$ | $2.40$ | $\text{deg/cm}$ |
| Integral Gain | $K_i$ | $0.04$ | $\text{deg}/(\text{cm}\cdot\text{s})$ |
| Derivative Gain | $K_d$ | $0.90$ | $\text{deg}\cdot\text{s}/\text{cm}$ |

---

## 🛒 Bill of Materials (BOM) & Hardware Specs

| Item | Component | Specification | Quantity | Primary Purpose |
| :---: | :--- | :--- | :---: | :--- |
| 1 | Microcontroller | Arduino Uno R3 (ATmega328P, 16 MHz) | 1 | Real-time PID loop & telemetry |
| 2 | Distance Transducer | HC-SR04 Ultrasonic (40 kHz, 2–400 cm) | 1 | Contactless ball position feedback |
| 3 | Actuator | SG90 9g Micro Servo (1.8 kg·cm torque) | 1 | Beam tilt angle positioning |
| 4 | Display | 16x2 HD44780 Parallel LCD | 1 | Autonomous standalone telemetry |
| 5 | Contrast Potentiometer | 10kΩ Rotary Trimpot | 1 | LCD V0 bias contrast adjustment |
| 6 | Track Rig | Lightweight U-channel aluminum beam (40 cm) | 1 | Smooth rolling track |
| 7 | Rolling Sphere | Solid Delrin / Steel Ball ($\varnothing = 25\text{ mm}$) | 1 | Plant inertial rolling mass |
| 8 | Push Linkage | Steel rod + Ball link / Nylon clevis | 1 | Rigid mechanical displacement transfer |

---

## 🔌 Hardware Wiring Diagram

### 1. Ultrasonic Sensor (HC-SR04)
* **VCC:** $5\text{V}$ (Arduino)
* **Trig:** Digital Pin `D9`
* **Echo:** Digital Pin `D10`
* **GND:** Ground

### 2. Micro Servo (SG90)
* **Red (VCC):** $5\text{V}$ (Use dedicated rail if servo twitches)
* **Brown/Black (GND):** Ground
* **Orange/Yellow (PWM):** Digital Pin `D6`

### 3. Standalone 16x2 LCD Display (Parallel HD44780)
| LCD Pin | Pin Label | Connected To | Notes |
| :---: | :---: | :---: | :--- |
| 1 | VSS | GND | Power Ground |
| 2 | VDD | 5V | Logic Power |
| 3 | V0 | Potentiometer Wiper | Contrast Control (0–5V) |
| 4 | RS | D2 | Register Select |
| 5 | RW | GND | Write Mode |
| 6 | E | D3 | Enable Strobe |
| 11 | D4 | D4 | Data Bit 4 (Nibble mode) |
| 12 | D5 | D5 | Data Bit 5 |
| 13 | D6 | D7 | Data Bit 6 |
| 14 | D7 | D8 | Data Bit 7 |
| 15 | A | 5V via 220Ω | LED Backlight Anode |
| 16 | K | GND | LED Backlight Cathode |

---

## 🚀 Modes of Operation & How to Run

### Mode 1: Interactive Desktop Physics Simulator (Python Tkinter)
Features a true interactive physics viewport (drag ball with mouse, click beam to set target, drag servo horn in manual mode), stacked 3-panel oscilloscope ($r/y$, error $e$, servo angle $u$ with shaded saturation limits), disturbance injection suite, live EMA filter toggles, and CSV hardware replay:
```bash
python simulator.py
```
* **Mouse Interactions:** Left-click & drag ball to toss it; click beam to set target setpoint.
* **Disturbance Suite:** Push Ball, Constant Wind, Sensor Dropout, Acoustic Noise Burst.
* **Shortcuts:** `Space` = Pause/Resume, `N` = Nudge, `R` = Reset.

---

### Mode 2: Flagship 5-Tab Web Bench (Web Serial API)
Hostable on GitHub Pages for live demonstration on any modern browser (Chrome, Edge, Opera) without installing Python:
* **File:** [`index.html`](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/index.html) or [`visual_interactive_bench.html`](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/visual_interactive_bench.html)
* **Features:**
  * **Tab 1: Dynamic Bench:** Vector field overlay (Velocity, Gravity, PID Thrust), interactive physics, disturbance injection.
  * **Tab 2: Stability Analysis:** Moving closed-loop poles, Routh-Hurwitz stability criterion, Bode plot with Gain/Phase margins.
  * **Tab 3: Controller Comparator:** 4-way comparative audit (P vs PD vs PI vs PID) with live trajectory overlays and export to CSV.
  * **Tab 4: Hardware Terminal:** Direct USB serial connection via Web Serial API (`115200` baud) with live ASCII telemetry.
  * **Tab 5: Technical Report:** Embedded system specifications, linkage derivations, and viva defense notes.

---

### Mode 3: Hardware Telemetry Suite & High-Speed CSV Logger
Connects to the physical Arduino over USB at 115200 baud, displaying real-time digital phosphor oscilloscope traces and logging synchronized telemetry:
```bash
python hardware_suite.py
```
* Logs `[timestamp, setpoint, distance, error, servo_angle, Kp, Ki, Kd]` to `logs/hardware_run_YYYYMMDD_HHMMSS.csv`.
* Automatic port scanner and auto-reconnect watchdog.

---

### Mode 4: Non-Linear System Identification Engine
Fits viscous damping, servo time constant $\tau$, and linkage kinematics to physical step response logs using SciPy `L-BFGS-B`:
```bash
python system_identification.py
```
* Generates `Docs/system_identification_fit.png` and `identified_plant_params.json`.
* Validates physical damping ($30.6\%$ error reduction over idealized physics).

---

### Mode 5: Comprehensive Control Stability & Transient Benchmark
Computes exact rise time ($t_r$), settling time ($t_s$), overshoot ($M_p$), steady-state error ($e_{ss}$), ITAE cost, and Bode stability margins:
```bash
python analysis_suite.py
```
* Automated unit verification:
```bash
python test_transient_metrics.py
```

---

## 🧪 Wokwi Virtual Hardware Simulation

### Honest Academic Disclaimer
> **Notice to Evaluators:** The Wokwi simulation ([`wokwi.toml`](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/wokwi.toml) / [`pid_control_main/diagram.json`](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/pid_control_main/diagram.json)) is provided **strictly as proof of embedded firmware execution on virtual AVR silicon (ATmega328P)**. It demonstrates cycle-accurate execution of the $33.3\text{ Hz}$ scheduler, non-blocking ultrasonic `pulseIn()` handling, LCD rendering, and EEPROM read/write routines. **All experimental validation and tuning claims in this capstone are verified exclusively using physical time-series CSV logs from the real ball-on-beam rig.**

To run the virtual circuit in Wokwi:
1. Open [Wokwi Arduino Uno Simulator](https://wokwi.com/).
2. Load [`pid_control_main/pid_control_main.ino`](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/pid_control_main/pid_control_main.ino) and [`pid_control_main/diagram.json`](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/pid_control_main/diagram.json).
3. Observe virtual servo angle changes in response to ultrasonic distance adjustments.

---

## 📈 System Identification & Hardware Validation

To guarantee rigorous scientific integrity, our simulator was calibrated against logged physical step responses:
* **Initial Physics RMSE:** $2.14\text{ cm}$
* **Calibrated Model RMSE:** $1.48\text{ cm}$ (**$30.6\%$ Improvement**)
* **Identified Servo Lag:** $\tau_{\text{servo}} \approx 65\text{ ms}$
* **Identified Viscous Friction:** $b \approx 0.082\text{ N}\cdot\text{s/m}$

![System Identification Calibration](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/Docs/system_identification_fit.png)

---

## 🎓 Comprehensive Viva Examination Guide

For in-depth theoretical proofs, pole-zero stability derivations, anti-windup explanations, and professor viva answers, consult:
* 📖 [**Viva Q&A Examination Cheatsheet**](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/Docs/Viva_CheatSheet.md)
* 📋 [**Physical Testing & Experimental Checklist**](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/Docs/Physical_Testing_Checklist.md)
* 📑 [**Full 100-Mark Capstone Engineering Report**](file:///c:/Users/EW0116TU/PID/PID-Ultrasonic-Tracking-System/Docs/Full_Project_Report.md)
