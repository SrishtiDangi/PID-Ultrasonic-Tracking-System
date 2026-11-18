# PHYSICAL EXPERIMENTAL TESTING & HARDWARE VALIDATION CHECKLIST
**Project:** Closed-Loop PID Ultrasonic Dynamic Tracking System  
**Target:** 100-Mark University Capstone Evaluation  
**Hardware Requirements:** Arduino Uno, HC-SR04, SG90 Servo, 16x2 LCD, USB Cable, PC running `hardware_suite.py`

---

## 1. PRE-FLIGHT BENCH SANITY CHECKS
Before recording data, execute these hardware verification steps:
1. **Mechanical Alignment:**
   - With power off, gently rotate SG90 horn to its physical center ($90^\circ$).
   - Ensure the balance beam sits horizontally level ($\alpha \approx 0.0^\circ$).
   - Confirm linkage pushrod moves smoothly without mechanical binding or gear slip.
2. **Sensor Alignment:**
   - Verify HC-SR04 transducers point directly down the centerline of the beam/track.
   - Clean the transducer mesh of any dust or obstructions.
3. **Power Decoupling:**
   - Ensure a $100\,\mu\text{F} - 470\,\mu\text{F}$ electrolytic capacitor is placed across the servo 5V and GND rail on your breadboard to prevent microcontroller brownout resets caused by servo inrush current.
4. **Serial Connection:**
   - Launch `python hardware_suite.py`.
   - Select your Arduino COM port and verify two-way connection @ `115200` baud.
   - Click **Sweep Left (45°)**, **Center (90°)**, and **Sweep Right (135°)** to confirm the servo responds.

---

## 2. STANDARD TEST MATRIX & DATA ACQUISITION

All tests are recorded using the **CSV Telemetry Logger** in `hardware_suite.py` which records at $\approx 33\,\text{Hz}$ ($30\,\text{ms}$ intervals):
$$\text{Columns: } \texttt{timestamp,setpoint,distance,error,servo\_angle,Kp,Ki,Kd}$$

### Test 1: Sensor Static Calibration & Noise Baseline
* **Objective:** Quantify HC-SR04 measurement variance, quantization noise, and verify EMA filter damping.
* **Procedure:**
  1. Fix a stationary target block at exactly $10.0\,\text{cm}$, $20.0\,\text{cm}$, and $30.0\,\text{cm}$ using a physical ruler.
  2. Click `🔴 Start Recording CSV` and capture $30\,\text{seconds}$ (approx. $1000$ points) at each position.
  3. Click `⏹ Stop Recording CSV`.
* **Deliverable:** Calculate mean $\mu$, standard deviation $\sigma$, and peak-to-peak quantization jitter $\Delta y$.

### Test 2: Open-Loop Plant Step Response (For System Identification)
* **Objective:** Capture the uncompensated open-loop rolling acceleration to fit plant damping ($\mu_{\text{visc}}$), Coulomb friction, and linkage gain ($K_{\text{linkage}}$).
* **Procedure:**
  1. Place the ball at rest at distance $x = 10.0\,\text{cm}$.
  2. Start CSV recording in `hardware_suite.py`.
  3. Send an open-loop servo step command: `S102\n` (tilts beam by $\approx +1.7^\circ$).
  4. Allow the ball to roll freely toward the end stop; stop recording before impact.
  5. Save as `logs/step_test_openloop.csv`.
* **Deliverable:** Feed this file into `python system_identification.py` to auto-fit the simulation plant parameters.

### Test 3: Closed-Loop Proportional-Only ($P$-Control) Benchmark
* **Objective:** Demonstrate steady-state offset and underdamped oscillations inherent to pure proportional action.
* **Gains:** $K_p = 1.80^\circ/\text{cm}$, $K_i = 0.00$, $K_d = 0.00$.
* **Procedure:**
  1. Set Target to $20.0\,\text{cm}$.
  2. Record step from $10.0\,\text{cm} \rightarrow 20.0\,\text{cm}$.
  3. Log for $20\,\text{seconds}$.
* **Expected Result:** Sustained oscillations or significant steady-state droop ($e_{ss} \ne 0$), proving why Integral action is theoretically mandatory.

### Test 4: Closed-Loop Nominal PID Step Response
* **Objective:** Benchmark baseline transient performance metrics ($t_r, t_s, M_p, e_{ss}, \text{ITAE}$).
* **Gains:** $K_p = 2.40^\circ/\text{cm}$, $K_i = 0.04^\circ/(\text{cm}\cdot\text{s})$, $K_d = 0.90^\circ\cdot\text{s}/\text{cm}$.
* **Procedure:**
  1. Balance ball at $15.0\,\text{cm}$.
  2. Start CSV recording.
  3. Send target step `T28.0\n`.
  4. Allow system to reach steady state ($e < \pm 0.5\,\text{cm}$ for $> 3\,\text{seconds}$).
  5. Stop recording and save as `logs/pid_step_15_to_28.csv`.
* **Expected Result:** Smooth convergence, settling time $t_s < 3.0\,\text{s}$, overshoot $M_p < 15\%$, steady-state error $e_{ss} < 0.5\,\text{cm}$.

### Test 5: Dynamic Trajectory Tracking (Sine Wave)
* **Objective:** Evaluate phase lag and tracking error under non-stationary excitation.
* **Trajectory:** $r(t) = 22.0 + 8.0 \sin(2\pi \cdot 0.20 t)\,\text{cm}$ ($f = 0.20\,\text{Hz}$).
* **Procedure:**
  1. Record $45\,\text{seconds}$ of continuous sine tracking.
  2. Measure phase lag in milliseconds and peak-to-peak tracking error.
  3. Save as `logs/pid_sine_tracking.csv`.

### Test 6: External Disturbance Rejection
* **Objective:** Quantify recovery time and anti-windup clamping behavior.
* **Procedure:**
  1. With system balanced at $20.0\,\text{cm}$, start CSV logger.
  2. After $5\,\text{seconds}$, physically tap/nudge the ball by $\approx +10.0\,\text{cm}$.
  3. Observe recovery curve and servo counter-torque.
  4. Save as `logs/pid_disturbance_rejection.csv`.

---

## 3. PHYSICAL VALIDATION INTEGRITY NOTICE
In accordance with academic evaluation standards:
* Never present simulated plots as hardware data.
* Always overlay physical CSV records using the simulator's **Replay Mode** (`python simulator.py --replay`) to showcase the quantitative Root-Mean-Square Error ($\text{RMSE}$) between physical hardware and numerical simulation.
