# PID Ultrasonic Tracking & Positioning System
## Project & Presentation Notes (Viva Cheatsheet)

---

### 1. 30-Second Elevator Pitch (What to say when introducing the project)
> *"Good morning/afternoon, Sir. Our project is a **Closed-Loop PID Distance Tracking & Balancing System**. It uses an HC-SR04 ultrasonic sensor to measure an object's distance in real time, computes the position error against a defined target, and processes it through a discrete PID control algorithm on the Arduino. The system then actuates a servo motor to minimize that error. We also implemented an Exponential Moving Average filter to cancel sensor acoustic noise, anti-windup to prevent integral saturation, and a live serial & LCD telemetry interface for dynamic software tuning."*

---

### 2. System Architecture & Block Diagram

```text
  [ Target Setpoint (e.g. 20cm) ]
                |
                v  (+)
       [ Error Calculation ] <--- ( - ) [ EMA Distance Filter (alpha=0.3) ]
                |                                      ^
                v                                      |
         [ PID Controller ]                   [ HC-SR04 Ultrasonic ]
        (P + I + D calculation)                 (Pulse duration / Echo)
                |                                      ^
                v                                      |
       [ Servo Motor Actuator ] ----------------> [ Physical Target ]
       (Center: 90°, Range: 20°-160°)
                |
                +---> [ 16x2 LCD Display Telemetry (Live T, D, E, S) ]
                +---> [ USB Serial Telemetry / Plotter (115200 Baud) ]
```

---

### 3. The Mathematics & Engineering Highlights

#### A. Error Formulation
$$\text{Error}(t) = \text{Target} - \text{Distance}_{\text{smoothed}}$$
* **Deadband:** If $|\text{Error}| < 1.0\text{ cm}$, Error is forced to $0$ to prevent microscopic servo jitter.

#### B. The PID Terms
1. **Proportional ($P = K_p \times \text{Error}$):**
   * Produces an actuation proportional to the current error.
   * *Role:* Gives the immediate corrective response.
2. **Integral ($I = K_i \times \sum \text{Error} \times \Delta t$):**
   * Eliminates persistent steady-state error (e.g. friction, gravity on the beam).
   * *Anti-Windup Clamping:* Clamped between $[-50, +50]$ to prevent actuator saturation and overshoot.
3. **Derivative ($D = K_d \times \frac{\Delta \text{Error}}{\Delta t}$):**
   * Predicts future error by looking at the rate of change.
   * Acts as an electronic damper to prevent overshooting and oscillations.
   * *Noise Filter:* HC-SR04 readings produce spikes. We pass the derivative through a low-pass filter ($\alpha = 0.2$) so the servo doesn't violently jerk.

---

### 4. Step-by-Step Live Demo Script for the Professor

1. **Step 1: Power On & Initialization**
   * Plug in Arduino USB to laptop.
   * Point out the **16x2 LCD** displaying `PID Initializing` and settling on `T:20.0` (Target) and `D:XX.X` (Current Distance).
2. **Step 2: Physical Disturbance Test**
   * Move your hand or a cardboard sheet towards the sensor ($10\text{ cm}$).
   * Show how the servo rotates in the opposite direction to push away / compensate.
   * Move your hand further ($30\text{ cm}$): Show the servo tilting in the reverse direction.
3. **Step 3: Software Telemetry & Serial Plotter**
   * In Arduino IDE, open **Tools > Serial Plotter** (`Ctrl + Shift + L`) at **115200 baud**.
   * Show the live graph on your laptop screen. Point out the red/blue lines tracking the target setpoint curve in real time!
4. **Step 4: Live Dynamic Software Tuning**
   * Open the **Serial Monitor**.
   * Type `T15` and hit Enter: Explain to the professor that the target distance was changed from 20 cm to 15 cm *on the fly without recompiling or uploading code*.
   * Watch the LCD instantly reflect `T:15.0`!
5. **Step 5: Failsafe Demonstration**
   * Pull your hand completely out of view.
   * Show that after 1 second of signal loss, the LCD cleanly indicates `SEARCHING / Out of Range`, and the servo returns safely to its center resting position ($90^\circ$).

---

### 5. Professor Viva Q&A (Top Questions & Crisp Answers)

* **Q1: Why did you use PID instead of simple ON/OFF (Bang-Bang) control?**
  * *Answer:* ON/OFF control causes constant overshoot and harsh oscillations around the target. PID provides smooth proportional control with derivative damping, settling gently at the exact distance.
* **Q2: Why is the HC-SR04 derivative noisy and how did you fix it?**
  * *Answer:* Ultrasonic distance discretizes into small jumps. Taking a derivative $(\Delta e / \Delta t)$ amplifies these rapid jumps into large spikes. We solved this with a discrete first-order Low-Pass Filter on the derivative term.
* **Q3: What is Integral Windup and how did you prevent it?**
  * *Answer:* If an error persists (e.g. object held by hand), the integral accumulator keeps summing up indefinitely. When released, it causes massive overshoot. We implemented Anti-Windup clamping ($[-50, +50]$).
* **Q4: What is the purpose of the Deadband?**
  * *Answer:* In mechanical systems, hunting for fractions of a millimeter wears out servo gears. A $1\text{ cm}$ deadband silences steady-state motor buzzing.
* **Q5: Why baud rate 115200 instead of standard 9600?**
  * *Answer:* At 9600 baud, printing long telemetry strings blocks the microcontroller CPU for ~10 ms per loop. At 115200 baud, serial TX takes $<1\text{ ms}$, ensuring our control loop runs at high frequency.

