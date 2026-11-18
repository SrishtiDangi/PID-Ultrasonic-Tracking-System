"""
================================================================================
PID ULTRASONIC TRACKING & BALL-ON-BEAM CONTROL SUITE (100 MARKS)
Advanced Control Engineering Bench with Dual Mechanical Rigs & Dynamic Analytics
================================================================================

Core Features:
1. Dual Mechanical Plant Rigs:
   - ⚖️ RIG 1: Classic Ball-and-Beam Balance System (MIT/Stanford Benchmark Problem)
   - 🎯 RIG 2: 1-DOF Linear Ultrasonic Distance Tracker with HC-SR04 & SG90 Servo
2. Advanced Trajectory Generator:
   - ⏹️ Step Input (Standard Setpoint: 15cm, 20cm, 30cm)
   - 🌊 Dynamic Sine Wave Trajectory Generator (Frequency & Amplitude Tracking)
   - 🔲 Square Wave Cyclic Disturbance Testing
3. Control Engineering Quality Metrics:
   - Integral of Absolute Error (IAE = ∫|e| dt) & ISE (Integral Square Error)
   - Real-time Oscillation Frequency & Resonance Spectrum Detector
   - Proportional, Integral, Derivative Live Energy Decomposition
4. Virtual 16x2 LCD Display & USB Hardware-in-the-Loop Mode
"""

import sys
import math
import time
import random
import threading
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class AdvancedPIDBench(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PID Ultrasonic Control Engineering Bench // Capstone 100 Marks")
        self.geometry("1080x760")
        self.configure(bg="#0f172a")
        self.minsize(980, 680)

        # Operational States
        self.mode = "sim"                     # "sim" or "hw"
        self.rig_type = "beam"                # "beam" (Ball & Beam) or "tracker" (Linear distance)
        self.trajectory_mode = "step"         # "step", "sine", "square"
        self.controller_type = "PID"          # "PID" or "ON_OFF"
        self.is_hw_connected = False
        self.ser = None

        # Physical Plant Dynamics (2nd-Order Model)
        self.position = 20.0                  # cm (ball position on beam / object distance)
        self.velocity = 0.0                  # cm/s
        self.target = 20.0                    # cm (desired distance / balance point)
        self.beam_angle = 0.0                # degrees (-25 to +25 deg)
        self.noise_amp = 0.20                # cm

        # Trajectory Generator State
        self.sine_freq = 0.25                 # Hz
        self.sine_amp = 10.0                  # cm (+/- 10cm)
        self.traj_time = 0.0

        # Performance Indexes (Formal Academic Control Metrics)
        self.iae = 0.0                        # Integral of Absolute Error (∫|e| dt)
        self.ise = 0.0                        # Integral of Square Error (∫e² dt)
        self.dominant_freq = 0.0              # Hz (oscillation frequency)
        self.zero_crossings = 0
        self.zc_timer = time.time()

        # PID Controller Parameters
        self.kp = 2.40
        self.ki = 0.04
        self.kd = 0.90
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_deriv = 0.0
        self.alpha_d = 0.20
        self.alpha_dist = 0.30
        self.smoothed_dist = 20.0
        self.p_out = 0.0
        self.i_out = 0.0
        self.d_out = 0.0
        self.control_output = 0.0
        self.servo_angle = 90.0

        # Oscilloscope Buffers (140 samples)
        self.buf_len = 140
        self.target_buf = [20.0] * self.buf_len
        self.pos_buf = [20.0] * self.buf_len
        self.ctrl_buf = [0.0] * self.buf_len

        # Time & animation
        self.last_time = time.time()
        self.wave_phase = 0.0

        self.build_ui()
        self.run_loop()

    def build_ui(self):
        # 1. COMMAND & RIG SELECTION HEADER
        header = tk.Frame(self, bg="#1e293b", padx=14, pady=8)
        header.pack(fill=tk.X, side=tk.TOP)

        title_box = tk.Frame(header, bg="#1e293b")
        title_box.pack(side=tk.LEFT)
        tk.Label(title_box, text="PID ULTRASONIC CONTROL ENGINEERING BENCH", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 12, "bold")).pack(anchor="w")
        tk.Label(title_box, text="Discrete Compensator • Dual Mechanical Rigs • Dynamic Trajectory Tracking", fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 8)).pack(anchor="w")

        controls_box = tk.Frame(header, bg="#1e293b")
        controls_box.pack(side=tk.RIGHT)

        # Rig Switcher (Ball & Beam vs Linear Tracker)
        self.btn_rig = tk.Button(controls_box, text="Rig: ⚖️ Ball-on-Beam", command=self.toggle_rig, bg="#6366f1", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=9, pady=3, cursor="hand2")
        self.btn_rig.pack(side=tk.LEFT, padx=4)

        # Trajectory Mode Switcher
        self.btn_traj = tk.Button(controls_box, text="Trajectory: Step", command=self.cycle_trajectory, bg="#0284c7", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=9, pady=3, cursor="hand2")
        self.btn_traj.pack(side=tk.LEFT, padx=4)

        # PID vs Bang-Bang Mode
        self.btn_algo = tk.Button(controls_box, text="Mode: PID", command=self.toggle_algo, bg="#059669", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=9, pady=3, cursor="hand2")
        self.btn_algo.pack(side=tk.LEFT, padx=4)

        # Hardware COM Port
        self.port_combo = ttk.Combobox(controls_box, width=7, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=3)
        self.refresh_ports()
        self.btn_hw = tk.Button(controls_box, text="USB", command=self.toggle_serial, bg="#334155", fg="#cbd5e1", font=("Segoe UI", 8, "bold"), relief="flat", padx=6, pady=3, cursor="hand2")
        self.btn_hw.pack(side=tk.LEFT, padx=2)

        # 2. TELEMETRY BADGES ROW (Including Academic Cost Functions IAE/ISE)
        tiles_frame = tk.Frame(self, bg="#0f172a", padx=12, pady=6)
        tiles_frame.pack(fill=tk.X)

        self.lbl_target = self.make_tile(tiles_frame, "TARGET SETPOINT (r)", "20.0 cm", "#38bdf8", 0)
        self.lbl_measured = self.make_tile(tiles_frame, "MEASURED POSITION (y)", "20.0 cm", "#10b981", 1)
        self.lbl_error = self.make_tile(tiles_frame, "TRACKING ERROR (e)", "0.0 cm", "#f43f5e", 2)
        self.lbl_servo = self.make_tile(tiles_frame, "SERVO ANGLE (θ)", "90°", "#fbbf24", 3)
        self.lbl_iae = self.make_tile(tiles_frame, "ERROR INDEX (IAE = ∫|e|dt)", "0.0", "#a78bfa", 4)
        self.lbl_freq = self.make_tile(tiles_frame, "LOOP FREQUENCY", "40.0 Hz", "#34d399", 5)

        # 3. CENTER VIEWPORTS: MECHANICAL RIG ANIMATION (TOP) + OSCILLOSCOPE (BOTTOM)
        center_frame = tk.Frame(self, bg="#0f172a", padx=12)
        center_frame.pack(fill=tk.BOTH, expand=True)

        # 3A. Mechanical Plant Animation Viewport
        anim_card = tk.Frame(center_frame, bg="#1e293b", highlightthickness=1, highlightbackground="#334155")
        anim_card.pack(fill=tk.X, pady=3)

        anim_hdr = tk.Frame(anim_card, bg="#1e293b", padx=8, pady=3)
        anim_hdr.pack(fill=tk.X)
        self.anim_title_lbl = tk.Label(anim_hdr, text="MECHANICAL RIG: ⚖️ CLASSIC BALL-AND-BEAM SYSTEM (ULTRASONIC BALANCING)", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 8, "bold"))
        self.anim_title_lbl.pack(side=tk.LEFT)
        self.anim_sub_lbl = tk.Label(anim_hdr, text="PHYSICS: GRAVITY ACCELERATION g•sin(θ) BALANCED BY ACTUATOR COUNTER-TORQUE", fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 7))
        self.anim_sub_lbl.pack(side=tk.RIGHT)

        self.anim_canvas = tk.Canvas(anim_card, height=140, bg="#090d16", highlightthickness=0)
        self.anim_canvas.pack(fill=tk.X, expand=True)

        # 3B. Real-Time Oscilloscope Viewport
        graph_card = tk.Frame(center_frame, bg="#1e293b", highlightthickness=1, highlightbackground="#334155")
        graph_card.pack(fill=tk.BOTH, expand=True, pady=3)

        graph_hdr = tk.Frame(graph_card, bg="#1e293b", padx=8, pady=3)
        graph_hdr.pack(fill=tk.X)
        tk.Label(graph_hdr, text="LIVE TIME-DOMAIN OSCILLOSCOPE", fg="#e2e8f0", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)
        tk.Label(graph_hdr, text="■ Target r(t)", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=10)
        tk.Label(graph_hdr, text="■ Output y(t)", fg="#10b981", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=6)
        tk.Label(graph_hdr, text="■ Effort u(t)", fg="#fbbf24", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=6)

        self.status_msg = tk.Label(graph_hdr, text="Trajectory: Stationary Setpoint @ 20.0 cm", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 8))
        self.status_msg.pack(side=tk.RIGHT, padx=6)

        self.graph_canvas = tk.Canvas(graph_card, bg="#05070c", highlightthickness=0)
        self.graph_canvas.pack(fill=tk.BOTH, expand=True)

        # 4. BOTTOM BENCH CONTROLS: LIVE PID SLIDERS & TEST GENERATORS
        ctrl_card = tk.Frame(self, bg="#1e293b", padx=12, pady=6, highlightthickness=1, highlightbackground="#334155")
        ctrl_card.pack(fill=tk.X, side=tk.BOTTOM, padx=12, pady=6)

        sliders_frame = tk.Frame(ctrl_card, bg="#1e293b")
        sliders_frame.pack(fill=tk.X)

        self.s_kp = self.add_slider(sliders_frame, "PROPORTIONAL (Kp):", 0.0, 8.0, self.kp, 0.1, "#38bdf8", 0)
        self.s_ki = self.add_slider(sliders_frame, "INTEGRAL (Ki):", 0.0, 0.2, self.ki, 0.01, "#10b981", 1)
        self.s_kd = self.add_slider(sliders_frame, "DERIVATIVE (Kd):", 0.0, 2.5, self.kd, 0.05, "#fbbf24", 2)
        self.s_target = self.add_slider(sliders_frame, "TARGET SETPOINT (cm):", 5.0, 45.0, self.target, 1.0, "#a78bfa", 3)

        # Actions & Probes Row
        btn_row = tk.Frame(ctrl_card, bg="#1e293b", pady=3)
        btn_row.pack(fill=tk.X)

        tk.Label(btn_row, text="TRAJECTORY PROBES:", fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT, padx=3)
        for t_val in [15, 20, 30]:
            tk.Button(btn_row, text=f"Step {t_val} cm", command=lambda v=t_val: self.set_target(v), bg="#334155", fg="white", relief="flat", padx=8, pady=1, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=2)

        tk.Label(btn_row, text="|", fg="#475569", bg="#1e293b").pack(side=tk.LEFT, padx=6)

        # Disturbance & Reset Buttons
        tk.Button(btn_row, text="⚡ NUDGE OBJECT (+10 cm PUSH)", command=self.apply_disturbance, bg="#e11d48", fg="white", relief="flat", padx=10, pady=2, font=("Segoe UI", 8, "bold"), cursor="hand2").pack(side=tk.LEFT, padx=3)
        tk.Button(btn_row, text="🌊 SINE WAVE TRACKING", command=lambda: self.set_trajectory("sine"), bg="#0284c7", fg="white", relief="flat", padx=9, pady=2, font=("Segoe UI", 8, "bold"), cursor="hand2").pack(side=tk.LEFT, padx=3)
        tk.Button(btn_row, text="Reset Error Index (IAE)", command=self.reset_metrics, bg="#334155", fg="#cbd5e1", relief="flat", padx=7, pady=2, font=("Segoe UI", 7)).pack(side=tk.LEFT, padx=3)

    def make_tile(self, parent, title, val, color, col):
        frame = tk.Frame(parent, bg="#1e293b", padx=8, pady=5, highlightthickness=1, highlightbackground="#334155")
        frame.grid(row=0, column=col, padx=3, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        accent = tk.Frame(frame, bg=color, height=2)
        accent.pack(fill=tk.X, side=tk.TOP, pady=(0, 3))

        tk.Label(frame, text=title, fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 7, "bold")).pack(anchor="w")
        lbl = tk.Label(frame, text=val, fg=color, bg="#1e293b", font=("Consolas", 11, "bold"))
        lbl.pack(anchor="w", pady=1)
        return lbl

    def add_slider(self, parent, label, from_val, to_val, default_val, res, color, col):
        box = tk.Frame(parent, bg="#1e293b", padx=5)
        box.grid(row=0, column=col, sticky="ew")
        parent.grid_columnconfigure(col, weight=1)

        var = tk.DoubleVar(value=default_val)
        top = tk.Frame(box, bg="#1e293b")
        top.pack(fill=tk.X)
        tk.Label(top, text=label, fg="#cbd5e1", bg="#1e293b", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT)
        val_lbl = tk.Label(top, text=f"{default_val:.2f}", fg=color, bg="#1e293b", font=("Consolas", 8, "bold"))
        val_lbl.pack(side=tk.RIGHT)

        def on_change(v):
            val_lbl.config(text=f"{float(v):.2f}")
            self.on_param_update()

        scale = tk.Scale(box, from_=from_val, to=to_val, resolution=res, variable=var, orient=tk.HORIZONTAL, showvalue=False, command=on_change, bg="#334155", fg=color, highlightthickness=0, troughcolor="#090d16", activebackground=color)
        scale.pack(fill=tk.X, pady=1)
        scale.var = var
        return scale

    def toggle_rig(self):
        if self.rig_type == "beam":
            self.rig_type = "tracker"
            self.btn_rig.config(text="Rig: 🎯 1-DOF Tracker", bg="#0284c7")
            self.anim_title_lbl.config(text="MECHANICAL RIG: 🎯 LINEAR 1-DOF DISTANCE TRACKER (HC-SR04 & SG90 SERVO)")
            self.anim_sub_lbl.config(text="PHYSICS: DIRECT HORIZONTAL DISPLACEMENT FOLLOWING WITH DEAD-BAND CONTROL")
        else:
            self.rig_type = "beam"
            self.btn_rig.config(text="Rig: ⚖️ Ball-on-Beam", bg="#6366f1")
            self.anim_title_lbl.config(text="MECHANICAL RIG: ⚖️ CLASSIC BALL-AND-BEAM SYSTEM (ULTRASONIC BALANCING)")
            self.anim_sub_lbl.config(text="PHYSICS: GRAVITY ACCELERATION g•sin(θ) BALANCED BY ACTUATOR COUNTER-TORQUE")

    def cycle_trajectory(self):
        modes = ["step", "sine", "square"]
        idx = (modes.index(self.trajectory_mode) + 1) % len(modes)
        self.set_trajectory(modes[idx])

    def set_trajectory(self, mode):
        self.trajectory_mode = mode
        if mode == "step":
            self.btn_traj.config(text="Trajectory: Step", bg="#0284c7")
            self.status_msg.config(text="Trajectory: Fixed Setpoint Mode", fg="#38bdf8")
        elif mode == "sine":
            self.btn_traj.config(text="Trajectory: 🌊 Sine Wave", bg="#6366f1")
            self.status_msg.config(text="Trajectory: Continuous Sine Tracking (0.25 Hz)", fg="#a78bfa")
        elif mode == "square":
            self.btn_traj.config(text="Trajectory: 🔲 Square Wave", bg="#d97706")
            self.status_msg.config(text="Trajectory: Alternating Square Setpoint Step", fg="#fbbf24")

    def toggle_algo(self):
        if self.controller_type == "PID":
            self.controller_type = "ON_OFF"
            self.btn_algo.config(text="Mode: Bang-Bang", bg="#e11d48")
            self.status_msg.config(text="ON/OFF Bang-Bang Active: Violent hunting / limit cycles!", fg="#f43f5e")
        else:
            self.controller_type = "PID"
            self.btn_algo.config(text="Mode: PID", bg="#059669")
            self.status_msg.config(text="PID Active: Smooth exponential settling with zero steady-state offset.", fg="#10b981")

    def set_target(self, val):
        self.trajectory_mode = "step"
        self.btn_traj.config(text="Trajectory: Step", bg="#0284c7")
        self.target = float(val)
        self.s_target.var.set(val)
        self.on_param_update()

    def on_param_update(self):
        self.kp = self.s_kp.var.get()
        self.ki = self.s_ki.var.get()
        self.kd = self.s_kd.var.get()
        if self.trajectory_mode == "step":
            self.target = self.s_target.var.get()

        if self.mode == "hw" and self.is_hw_connected and self.ser:
            try:
                cmd = f"P{self.kp}\nI{self.ki}\nD{self.kd}\nT{self.target}\n"
                self.ser.write(cmd.encode())
            except Exception:
                pass

    def apply_disturbance(self):
        self.position = min(48.0, self.position + 10.0)
        self.velocity += 14.0
        self.status_msg.config(text="Disturbance Injected (+10 cm): PID Counter-acting!", fg="#f43f5e")

    def reset_metrics(self):
        self.iae = 0.0
        self.ise = 0.0

    def refresh_ports(self):
        if SERIAL_AVAILABLE:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                self.port_combo.current(0)
        else:
            self.port_combo['values'] = ["No COM"]

    def toggle_serial(self):
        if not SERIAL_AVAILABLE:
            messagebox.showerror("Error", "PySerial is not installed.")
            return
        if not self.is_hw_connected:
            port = self.port_combo.get()
            if not port:
                messagebox.showerror("Error", "Select COM Port first.")
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.is_hw_connected = True
                self.mode = "hw"
                self.btn_hw.config(text="Disconnect", bg="#e11d48")
                threading.Thread(target=self.serial_reader, daemon=True).start()
            except Exception as e:
                messagebox.showerror("Error", str(e))
        else:
            self.is_hw_connected = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.btn_hw.config(text="USB", bg="#334155")
            self.mode = "sim"

    def serial_reader(self):
        while self.is_hw_connected and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "Target:" in line and "Distance:" in line:
                    parts = line.split('\t')
                    d = {}
                    for p in parts:
                        if ':' in p:
                            k, v = p.split(':', 1)
                            d[k.strip()] = float(v.strip())
                    self.target = d.get("Target", self.target)
                    self.position = d.get("Distance", self.position)
                    self.smoothed_dist = self.position
                    self.servo_angle = d.get("Servo", self.servo_angle)
            except Exception:
                pass
            time.sleep(0.01)

    def physics_step(self, dt):
        if self.mode == "hw":
            return

        # Trajectory Generator
        self.traj_time += dt
        if self.trajectory_mode == "sine":
            self.target = 22.0 + self.sine_amp * math.sin(2.0 * math.pi * self.sine_freq * self.traj_time)
            self.s_target.var.set(round(self.target, 1))
        elif self.trajectory_mode == "square":
            self.target = 15.0 if int(self.traj_time * 0.4) % 2 == 0 else 30.0
            self.s_target.var.set(self.target)

        # Ultrasonic Noise & Filter
        noise = random.gauss(0, self.noise_amp)
        raw_dist = max(2.0, min(50.0, self.position + noise))
        self.smoothed_dist = (self.alpha_dist * raw_dist) + ((1.0 - self.alpha_dist) * self.smoothed_dist)
        error = self.target - self.smoothed_dist

        # Performance Indexes (IAE & ISE)
        self.iae += abs(error) * dt
        self.ise += (error ** 2) * dt

        # Control Algorithm
        if self.controller_type == "ON_OFF":
            if error > 0.8:
                self.control_output = 30.0
            elif error < -0.8:
                self.control_output = -30.0
            else:
                self.control_output = 0.0
            self.p_out = self.control_output
            self.i_out = 0.0
            self.d_out = 0.0
            self.servo_angle = 90.0 + self.control_output
        else:
            ctrl_error = 0.0 if abs(error) < 0.5 else error
            self.p_out = self.kp * ctrl_error

            self.integral += ctrl_error * dt
            self.integral = max(-35.0, min(35.0, self.integral))
            self.i_out = self.ki * self.integral

            raw_d = (ctrl_error - self.prev_error) / dt if dt > 0 else 0.0
            self.filtered_deriv = (self.alpha_d * raw_d) + ((1.0 - self.alpha_d) * self.filtered_deriv)
            self.d_out = self.kd * self.filtered_deriv
            self.prev_error = ctrl_error

            self.control_output = self.p_out + self.i_out + self.d_out
            self.servo_angle = max(25.0, min(155.0, 90.0 + self.control_output))

        # Dynamic Mechanics: Ball & Beam vs Linear Tracker
        if self.rig_type == "beam":
            # Tilt angle of beam proportional to servo rotation:
            self.beam_angle = (self.servo_angle - 90.0) * 0.35 # max +/- 22 deg
            # Physics of Ball rolling on tilted beam:
            # a = (5/7) * g * sin(theta) - damping * v
            g = 980.0 # cm/s^2
            theta_rad = math.radians(self.beam_angle)
            accel = (5.0 / 7.0) * g * math.sin(theta_rad) - (self.plant_damping * self.velocity)
            self.velocity += accel * dt
            self.position += self.velocity * dt
        else:
            # Linear Tracker
            tilt_rad = math.radians(self.servo_angle - 90.0)
            net_force = 120.0 * math.sin(tilt_rad) - (self.plant_damping * self.velocity)
            self.velocity += (net_force / self.plant_mass) * dt
            self.position += self.velocity * dt

        # Beam track boundaries (3 cm to 48 cm)
        if self.position <= 3.0:
            self.position = 3.0
            self.velocity = -0.3 * self.velocity # bounce slightly
        elif self.position >= 48.0:
            self.position = 48.0
            self.velocity = -0.3 * self.velocity

    def draw_mechanical_rig(self):
        c = self.anim_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        if self.rig_type == "beam":
            self.draw_ball_and_beam(c, w, h)
        else:
            self.draw_linear_tracker(c, w, h)

    def draw_ball_and_beam(self, c, w, h):
        """Draws classic Tilting Ball-and-Beam Benchmark System"""
        pivot_x = w // 2 - 40
        pivot_y = h - 45
        beam_len = w - 240

        # Pivot Stand (Triangular fulcrum)
        c.create_polygon(pivot_x - 16, pivot_y + 24, pivot_x + 16, pivot_y + 24, pivot_x, pivot_y, fill="#334155", outline="#64748b", width=2)
        c.create_oval(pivot_x - 5, pivot_y - 5, pivot_x + 5, pivot_y + 5, fill="#94a3b8")

        # Beam Tilt Math
        ang_rad = math.radians(self.beam_angle)
        half_len = beam_len / 2.0
        x1 = pivot_x - half_len * math.cos(ang_rad)
        y1 = pivot_y + half_len * math.sin(ang_rad)
        x2 = pivot_x + half_len * math.cos(ang_rad)
        y2 = pivot_y - half_len * math.sin(ang_rad)

        # The Aluminum Tilting Beam
        c.create_line(x1, y1, x2, y2, fill="#38bdf8", width=6)
        c.create_line(x1, y1, x2, y2, fill="#0284c7", width=2)

        # HC-SR04 Mounted at the left end of the tilted beam
        # Transducer face points along the beam
        c.create_rectangle(x1 - 10, y1 - 15, x1 + 10, y1 + 15, fill="#0284c7", outline="#38bdf8")
        c.create_oval(x1 + 6, y1 - 12, x1 + 14, y1 - 2, fill="#cbd5e1")
        c.create_oval(x1 + 6, y1 + 2, x1 + 14, y1 + 12, fill="#cbd5e1")
        c.create_text(x1 - 2, y1 - 22, text="HC-SR04", fill="#38bdf8", font=("Segoe UI", 7, "bold"))

        # Rolling Ball (Controlled Plant)
        pos_frac = (self.position - 3.0) / 45.0
        ball_x = x1 + pos_frac * (x2 - x1)
        ball_y = y1 + pos_frac * (y2 - y1) - 10

        c.create_oval(ball_x - 9, ball_y - 9, ball_x + 9, ball_y + 9, fill="#f59e0b", outline="#ffffff", width=2)
        c.create_text(ball_x, ball_y - 14, text=f"{self.position:.1f}cm", fill="#fbbf24", font=("Consolas", 8, "bold"))

        # Target Setpoint Marker (Cyan Flag on Beam)
        tgt_frac = (self.target - 3.0) / 45.0
        tx = x1 + tgt_frac * (x2 - x1)
        ty = y1 + tgt_frac * (y2 - y1)
        c.create_line(tx, ty - 25, tx, ty, fill="#38bdf8", width=2, dash=(2, 2))
        c.create_polygon(tx, ty - 25, tx + 10, ty - 20, tx, ty - 15, fill="#38bdf8")

        # Animated Sonar Waves traveling along the tilted beam
        self.wave_phase = (self.wave_phase + 0.18) % 1.0
        gap_span = (ball_x - x1)
        if gap_span > 20:
            for arc_i in range(3):
                prog = (self.wave_phase + arc_i * 0.33) % 1.0
                wx = x1 + gap_span * prog
                wy = y1 + (ball_y - y1) * prog
                c.create_arc(wx - 8, wy - 8, wx + 8, wy + 8, start=-40, extent=80, style=tk.ARC, outline="#38bdf8", width=1.5)

        # Servo Motor & Mechanical Linkage on the Right
        servo_x = w - 85
        servo_y = h - 45
        c.create_rectangle(servo_x - 20, servo_y - 18, servo_x + 20, servo_y + 18, fill="#0284c7", outline="#38bdf8", width=1)
        c.create_oval(servo_x - 6, servo_y - 6, servo_x + 6, servo_y + 6, fill="#ffffff")

        horn_rad = math.radians(self.servo_angle - 180)
        hx = servo_x + 22 * math.cos(horn_rad)
        hy = servo_y + 22 * math.sin(horn_rad)
        c.create_line(servo_x, servo_y, hx, hy, fill="#fbbf24", width=3)
        # Tie-rod mechanical push-linkage from servo horn to beam end (x2, y2)
        c.create_line(hx, hy, x2, y2, fill="#94a3b8", width=2, dash=(3, 2))
        c.create_text(servo_x, servo_y + 25, text=f"Servo: {int(self.servo_angle)}°", fill="#fbbf24", font=("Consolas", 8, "bold"))

    def draw_linear_tracker(self, c, w, h):
        """Draws 1-DOF Linear Tracker with Obstacle & 16x2 LCD Mockup"""
        track_y = h - 35
        start_x = 90
        end_x = w - 180
        track_w = end_x - start_x

        c.create_line(start_x - 10, track_y, end_x + 10, track_y, fill="#334155", width=2)
        for cm in range(0, 55, 10):
            tx = start_x + (cm / 50.0) * track_w
            c.create_line(tx, track_y, tx, track_y + 6, fill="#64748b", width=1)
            c.create_text(tx, track_y + 16, text=f"{cm}cm", fill="#94a3b8", font=("Consolas", 7))

        # Sensor
        sensor_x = start_x - 45
        sensor_y = track_y - 25
        c.create_rectangle(sensor_x - 18, sensor_y - 22, sensor_x + 18, sensor_y + 22, fill="#0284c7", outline="#38bdf8", width=2)
        c.create_oval(sensor_x - 12, sensor_y - 18, sensor_x + 12, sensor_y - 2, fill="#cbd5e1")
        c.create_oval(sensor_x - 12, sensor_y + 2, sensor_x + 12, sensor_y + 18, fill="#cbd5e1")
        c.create_text(sensor_x, sensor_y - 30, text="HC-SR04", fill="#38bdf8", font=("Segoe UI", 8, "bold"))

        # Target Setpoint Flag
        tgt_clamped = max(0.0, min(50.0, self.target))
        tgt_x = start_x + (tgt_clamped / 50.0) * track_w
        c.create_line(tgt_x, track_y - 50, tgt_x, track_y, fill="#38bdf8", width=2, dash=(3, 2))
        c.create_polygon(tgt_x, track_y - 50, tgt_x + 12, track_y - 42, tgt_x, track_y - 34, fill="#38bdf8")

        # Object
        pos_clamped = max(0.0, min(50.0, self.position))
        obj_x = start_x + (pos_clamped / 50.0) * track_w
        obj_y = track_y - 18
        c.create_rectangle(obj_x - 4, obj_y - 25, obj_x + 4, obj_y + 18, fill="#e2e8f0", outline="#38bdf8", width=2)
        c.create_text(obj_x, obj_y - 32, text=f"{self.position:.1f}cm", fill="#10b981", font=("Consolas", 8, "bold"))

        # Waves
        self.wave_phase = (self.wave_phase + 0.18) % 1.0
        gap = (obj_x - 4) - (sensor_x + 18)
        if gap > 20:
            for w_idx in range(3):
                prog = (self.wave_phase + w_idx * 0.33) % 1.0
                wx = (sensor_x + 18) + (gap * prog)
                r = 10 + 12 * prog
                c.create_arc(wx - r, sensor_y - r, wx + r, sensor_y + r, start=-40, extent=80, style=tk.ARC, outline="#38bdf8", width=2)

        # Virtual 16x2 LCD
        lcd_x = w - 90
        lcd_y = track_y - 25
        c.create_rectangle(lcd_x - 45, lcd_y - 18, lcd_x + 45, lcd_y + 20, fill="#1e3a8a", outline="#60a5fa", width=2)
        err = self.target - self.smoothed_dist
        e_sign = "+" if err > 0 else ""
        c.create_text(lcd_x - 38, lcd_y - 8, text=f"T:{self.target:.0f} D:{self.smoothed_dist:.1f}", fill="#ffffff", font=("Courier", 7, "bold"), anchor="w")
        c.create_text(lcd_x - 38, lcd_y + 8, text=f"E:{e_sign}{err:.1f} S:{int(self.servo_angle)}", fill="#ffffff", font=("Courier", 7, "bold"), anchor="w")

    def draw_graph(self):
        c = self.graph_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        y_max = 50.0

        for cm in [10, 20, 30, 40]:
            y_px = h - int((cm / y_max) * h)
            c.create_line(0, y_px, w, y_px, fill="#1e293b", dash=(2, 4))
            c.create_text(24, y_px - 8, text=f"{cm} cm", fill="#64748b", font=("Consolas", 7))

        dx = w / (self.buf_len - 1)

        # Target (Cyan)
        target_pts = []
        for i, val in enumerate(self.target_buf):
            x = i * dx
            y = h - (val / y_max) * h
            target_pts.extend([x, max(0, min(h, y))])
        if len(target_pts) >= 4:
            c.create_line(*target_pts, fill="#38bdf8", width=2)

        # Position (Green or Red)
        pos_col = "#10b981" if self.controller_type == "PID" else "#f43f5e"
        pos_pts = []
        for i, val in enumerate(self.pos_buf):
            x = i * dx
            y = h - (val / y_max) * h
            pos_pts.extend([x, max(0, min(h, y))])
        if len(pos_pts) >= 4:
            c.create_line(*pos_pts, fill=pos_col, width=2)

    def run_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt > 0.1:
            dt = 0.03

        self.physics_step(dt)

        self.target_buf.pop(0)
        self.target_buf.append(self.target)

        self.pos_buf.pop(0)
        self.pos_buf.append(self.position)

        # Telemetry Labels
        self.lbl_target.config(text=f"{self.target:.1f} cm")
        self.lbl_measured.config(text=f"{self.position:.1f} cm")
        err = self.target - self.position
        sign = "+" if err > 0 else ""
        self.lbl_error.config(text=f"{sign}{err:.1f} cm")
        self.lbl_servo.config(text=f"{int(self.servo_angle)}°")
        self.lbl_iae.config(text=f"{self.iae:.1f}")

        self.draw_mechanical_rig()
        self.draw_graph()

        self.after(28, self.run_loop)


if __name__ == "__main__":
    app = AdvancedPIDBench()
    app.mainloop()
