# PID Ultrasonic Tracking & Positioning System

A closed-loop distance control system. An **HC-SR04 ultrasonic sensor** measures the distance to an object, an **Arduino** runs a **PID controller**, and a **servo** moves to bring the measured distance to the target (for example 20 cm). The repository also contains a **Python physics simulator** for visualising and tuning the controller with or without hardware.

> Status: firmware v2 and the simulator are included. See [Results](#results) for measured performance (fill in with your own test data).

---

## Contents

- [Features](#features)
- [How it works](#how-it-works)
- [Hardware](#hardware)
- [Repository structure](#repository-structure)
- [Quick start](#quick-start)
- [Serial commands](#serial-commands)
- [Simulator](#simulator)
- [Tuning guide](#tuning-guide)
- [Results](#results)
- [Troubleshooting](#troubleshooting)
- [Limitations and future work](#limitations-and-future-work)

---

## Features

**Firmware (Arduino)**

- PID control with **derivative on measurement** (no kick when the target changes)
- **Median-of-3 + EMA** distance filtering, plus a low-pass filter on the derivative
- **Anti-windup**: the integral term is clamped in output units and stops accumulating while the servo is saturated
- **Fixed 60 ms sample time**, matching the HC-SR04 timing limits
- **Soft deadband** to prevent servo jitter near the target
- **Failsafe**: if no valid echo arrives for 1 s, the servo returns to centre and the controller state is reset
- **Live tuning over serial** (`P`, `I`, `D`, `T` commands) with range checking
- **Save/restore settings in EEPROM**
- Optional temperature compensation of the speed of sound
- Non-blocking 16x2 LCD telemetry (target, distance, error, servo angle)
- Serial log compatible with the Arduino Serial Plotter and `gui_monitor.py`

**Simulator (Python / Tkinter)**

- Second-order physical plant with sensor noise
- **PID vs ON/OFF (Bang-Bang)** comparison
- Three visual environments: greenhouse canopy tracking, vehicle adaptive cruise control, lab bench
- Live P, I, D contribution bars, step-response graph, and metrics (rise time, settling time, overshoot, steady-state error)
- Step buttons, a disturbance injector, and a live USB link to the real hardware

---

## How it works

```
 Target distance ──►(+)──► PID controller ──► Servo / actuator ──► Plant
                     ▲(-)                                            │
                     │                                               │
              Distance filter ◄── HC-SR04 sensor ◄───────────────────┘
        (median-of-3, then EMA)
```

Each cycle (every 60 ms):

1. **Measure**: the HC-SR04 sends a pulse and times the echo. `distance = echo_time × speed_of_sound / 2`.
2. **Filter**: median of the last 3 readings, then an exponential moving average (`alpha = 0.3`).
3. **Error**: `error = target − distance`, with a small deadband.
4. **PID**: `output = Kp·e + Ki·∫e + Kd·(rate of change)`.
5. **Actuate**: `servo angle = 90° + output`, limited to **20°–160°**.
6. **Display**: update the LCD and send telemetry over serial.

| Term | Role | Too much |
|------|------|----------|
| **P** | Pushes proportionally to the current error | Overshoot, oscillation |
| **I** | Removes small leftover error | Slow hunting, windup |
| **D** | Damps the motion (braking) | Amplifies sensor noise |

---

## Hardware

| Component | Notes |
|-----------|-------|
| Arduino Uno / Nano (ATmega328P) | Runs the controller |
| HC-SR04 ultrasonic sensor | Distance measurement, valid range 2–250 cm in firmware |
| Hobby servo | Actuator, 20°–160° safe range |
| 16x2 LCD (parallel, HD44780) | Live telemetry |
| USB cable | Power, upload, serial |

### Pin connections

| Signal | Arduino pin |
|--------|-------------|
| HC-SR04 Trig | 9 |
| HC-SR04 Echo | 10 |
| Servo signal | 6 |
| LCD RS | 2 |
| LCD E | 3 |
| LCD D4 | 4 |
| LCD D5 | 5 |
| LCD D6 | 7 |
| LCD D7 | 8 |

Power the HC-SR04 from 5 V and GND. For the servo, use a separate 5 V supply if it causes resets or noisy readings, and connect all grounds together.

---

## Repository structure

```
PID-Ultrasonic-Tracking-System/
├── pid_control_main/
│   └── pid_control_main.ino   Main firmware (v2)
├── lcd_test/                  LCD wiring test sketch
├── simulator.py               Physics simulator and tuning dashboard
├── gui_monitor.py             Python monitor for the serial telemetry
├── Docs/                      Project notes and viva cheat-sheet
├── README.md
└── .gitignore
```

---

## Quick start

### 1. Firmware

1. Install the [Arduino IDE](https://www.arduino.cc/en/software). The `Servo`, `LiquidCrystal`, and `EEPROM` libraries are included.
2. (Optional) Upload `lcd_test` first to confirm the LCD wiring.
3. Open `pid_control_main/pid_control_main.ino`.
4. Select your board and port under **Tools**, then **Verify** and **Upload**.
5. Open the Serial Monitor at **115200 baud** to see the telemetry.

> The red `#include` errors that VS Code may show are IntelliSense configuration issues, not code errors. If the Arduino IDE compiles it, the code is fine.

### 2. Simulator

Requirements: Python 3.8+ with Tkinter (included with the standard Windows and macOS installers). `pyserial` is only needed for the hardware link.

```bash
pip install pyserial        # optional, for the USB link
python simulator.py
```

### 3. Live monitor (optional)

Close the Arduino Serial Monitor first, because only one program can use the port at a time.

```bash
python gui_monitor.py
```

---

## Serial commands

Send at **115200 baud**. Commands work with or without a line ending.

| Command | Effect | Range |
|---------|--------|-------|
| `P<value>` | Set Kp (example: `P2.5`) | 0 – 100 |
| `I<value>` | Set Ki (resets the integral) | 0 – 10 |
| `D<value>` | Set Kd | 0 – 50 |
| `T<value>` | Set target distance in cm | 5 – 200 |
| `S` | Save Kp, Ki, Kd, target to EEPROM | |
| `R` | Restore default settings | |
| `?` | Print current settings | |

### Telemetry format

One tab-separated line per sample, readable by the Serial Plotter:

```
Target:20.00  Distance:19.80  Error:0.20  P:0.00  I:0.00  D:0.00  Output:0.00  Servo:90
```

`Error` is the true error (`target − distance`), before the deadband is applied.

### Default settings

| Parameter | Value |
|-----------|-------|
| Kp | 2.0 |
| Ki | 0.02 |
| Kd | 0.1 |
| Target | 20 cm |
| Sample time | 60 ms |
| Distance EMA alpha | 0.3 |
| Derivative filter alpha | 0.2 |
| Deadband | 0.5 cm |
| Servo range | 20° – 160° (centre 90°) |

---

## Simulator

`simulator.py` is a Tkinter application that simulates the closed loop and displays it live.

**Controls**

| Control | Function |
|---------|----------|
| `MODE` button | Switch between **PID** and **ON/OFF (Bang-Bang)** |
| `ENV` button | Cycle **Greenhouse → Vehicle ACC → Lab bench** |
| `SYSTEM` button | Energise or de-energise the actuator |
| Kp, Ki, Kd sliders | Tune the controller (ranges 0–10, 0–0.5, 0–3) |
| Target slider | Set the target distance, 5–45 cm |
| Noise slider | Sensor noise amplitude, 0–2 cm |
| Step buttons | Step the target to 15, 25, or 35 cm |
| Disturbance button | Inject a +8 cm physical nudge |
| `USB` button | Connect to the Arduino and display live hardware data |

**Displays**

- Status cards: target, distance, error, algorithm state, settling time, peak overshoot
- P, I, D action bars showing each term's contribution
- Step-response graph with setpoint, output, and effort traces
- Metrics: rise time (tr), settling time (ts), overshoot (Mp), steady-state error (e_ss)

**Plant model (simulation):** a second-order system with mass 1.0 and damping 1.8, driven by a force proportional to the servo tilt. The simulator's controller uses an EMA distance filter (0.3), a derivative low-pass filter (0.22), a 0.6 cm deadband, and an anti-windup clamp on the integral. The ON/OFF mode switches the actuator between ±35 with a 0.8 cm deadband.

> **Note:** the simulator is a model, not a measurement of the real hardware. Its default gains (Kp 2.4, Ki 0.05, Kd 0.85) and its output units differ from the firmware defaults, so tune the real system separately using the serial commands.

### Why PID instead of ON/OFF?

| | ON/OFF (Bang-Bang) | PID |
|---|---|---|
| Output | Only full +/− | Smoothly varying |
| Near the target | Keeps swinging (limit cycle) | Settles and holds |
| Accuracy | Poor | Good |
| Actuator wear | High | Low |

---

## Tuning guide

Start from the defaults and change **one value at a time**.

1. Set `I0` and `D0`. Raise `P` until the response is fast but starts to overshoot slightly.
2. Raise `D` until the overshoot is reduced and the motion is smooth. Stop if the servo becomes jittery.
3. Add a small `I` until the leftover error disappears.
4. If it oscillates, reduce `P` or `D`. Save the final values with `S`.

| Symptom | Likely fix |
|---------|-----------|
| Slow to reach the target | Increase Kp |
| Overshoots and oscillates | Reduce Kp, increase Kd |
| Stops slightly off target | Increase Ki (and check the deadband) |
| Servo jitters | Reduce Kd, increase the deadband |

---

## Results

<!-- Fill this section with your own measured data. -->

| Test | Setting | Rise time | Settling time | Overshoot | Steady-state error |
|------|---------|-----------|---------------|-----------|--------------------|
| PID (hardware) | Kp = ?, Ki = ?, Kd = ? | | | | |
| PID (simulator) | Kp 2.4, Ki 0.05, Kd 0.85 | | | | |
| ON/OFF (simulator) | | | never settles | sustained | |

Add screenshots of the simulator and a short demo video here.

---

## Troubleshooting

| Problem | What to check |
|---------|---------------|
| Distance reads -1 or the LCD shows `SEARCHING` | Object out of range (under 2 cm or over 250 cm), Trig/Echo wires swapped, or the surface is angled or soft |
| Servo twitches or the Arduino resets | Use a separate servo supply and share the ground |
| `gui_monitor.py` cannot open the port | Close the Arduino Serial Monitor first |
| Commands are ignored | Use 115200 baud; send e.g. `P2.5` |
| VS Code shows `cannot open source file "Servo.h"` | IntelliSense setup only; compile with the Arduino IDE |
| Readings are slightly off | Set `AMBIENT_TEMP_C` in the firmware to your room temperature |

---

## Limitations and future work

- The HC-SR04 is noisy and depends on the target's surface and angle.
- The sensor has a minimum range of about 2 cm and a maximum of about 4 m.
- Fixed gains may not suit every operating condition.
- Ideas: logging to CSV with automatic metrics, a time-of-flight sensor comparison, auto-tuning (relay method), and validating the simulator model against recorded hardware step responses.

---

## License

Add a license of your choice (for example MIT) and update this section.

## Authors

<!-- Add your name(s) and institution. -->