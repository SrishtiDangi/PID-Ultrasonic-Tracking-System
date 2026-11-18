"""
================================================================================
PID ULTRASONIC DYNAMIC TRACKING BENCH // COMPACT & RESPONSIVE SUITE
Fits all laptop screens (zero taskbar cut-off) • Live on startup • No text overlap
================================================================================
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


class VisualPIDSimulatorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PID ULTRASONIC DYNAMIC TRACKING BENCH")
        # Compact height (620px) to guarantee it fits above Windows taskbar on any screen scaling!
        self.geometry("980x620")
        self.configure(bg="#090d16")
        self.minsize(900, 580)

        # Operational States
        self.mode = "sim"
        self.rig_type = "beam"                # "beam" (Ball & Beam) or "tracker" (1-DOF)
        self.trajectory_mode = "sine"         # Start with active sine trajectory so it's ALIVE on launch!
        self.controller_type = "PID"
        self.show_vectors = True
        self.is_hw_connected = False
        self.ser = None

        # Start with an initial offset so motion & vectors are visibly active immediately!
        self.position = 12.0                  # cm
        self.velocity = 0.0                  # cm/s
        self.acceleration = 0.0              # cm/s^2
        self.target = 22.0                    # cm
        self.beam_angle = 0.0                # degrees (-22° to +22°)
        self.ball_rotation = 0.0             # degrees
        self.plant_mass = 1.0                # kg
        self.plant_damping = 2.0             # N*s/cm
        self.noise_amp = 0.20                # cm

        # Trajectory Generator State
        self.traj_time = 0.0
        self.sine_freq = 0.20                 # Hz
        self.sine_amp = 8.0                   # cm
        self.iae = 0.0

        # PID Controller Parameters
        self.kp = 2.40
        self.ki = 0.04
        self.kd = 0.90
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_deriv = 0.0
        self.alpha_d = 0.20
        self.alpha_dist = 0.30
        self.smoothed_dist = 12.0
        self.control_output = 0.0
        self.servo_angle = 90.0

        # Oscilloscope Buffers (140 samples)
        self.buf_len = 140
        self.target_buf = [22.0] * self.buf_len
        self.pos_buf = [12.0] * self.buf_len

        # Visual Loop Variables
        self.last_time = time.time()
        self.wave_phase = 0.0

        self.build_ui()
        self.run_loop()

    def build_ui(self):
        # 1. HEADER (Compact & Clean)
        header = tk.Frame(self, bg="#131c2e", padx=12, pady=6, highlightthickness=1, highlightbackground="#1e293b")
        header.pack(fill=tk.X, side=tk.TOP, padx=10, pady=(6, 4))

        title_box = tk.Frame(header, bg="#131c2e")
        title_box.pack(side=tk.LEFT)

        h_title = tk.Frame(title_box, bg="#131c2e")
        h_title.pack(anchor="w")
        tk.Label(h_title, text="●", fg="#38bdf8", bg="#131c2e", font=("Segoe UI", 11)).pack(side=tk.LEFT, padx=(0, 4))
        tk.Label(h_title, text="PID ULTRASONIC DYNAMIC TRACKING BENCH", fg="#38bdf8", bg="#131c2e", font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT)

        tk.Label(title_box, text="HC-SR04 Transducer • SG90 Servo • Live Physics Vectors & Oscilloscope", fg="#94a3b8", bg="#131c2e", font=("Segoe UI", 8)).pack(anchor="w")

        controls_box = tk.Frame(header, bg="#131c2e")
        controls_box.pack(side=tk.RIGHT)

        self.btn_rig = tk.Button(controls_box, text="Rig: ⚖️ Ball-on-Beam", command=self.toggle_rig, bg="#4f46e5", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=8, pady=2, cursor="hand2")
        self.btn_rig.pack(side=tk.LEFT, padx=3)

        self.btn_mode = tk.Button(controls_box, text="Mode: PID", command=self.toggle_controller_mode, bg="#0284c7", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=8, pady=2, cursor="hand2")
        self.btn_mode.pack(side=tk.LEFT, padx=3)

        self.btn_traj = tk.Button(controls_box, text="Trajectory: 🌊 Sine", command=self.toggle_trajectory, bg="#059669", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=8, pady=2, cursor="hand2")
        self.btn_traj.pack(side=tk.LEFT, padx=3)

        self.port_combo = ttk.Combobox(controls_box, width=6, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=2)
        self.refresh_ports()
        self.btn_hw = tk.Button(controls_box, text="USB", command=self.toggle_serial, bg="#1e293b", fg="#94a3b8", font=("Segoe UI", 8, "bold"), relief="flat", padx=6, pady=2, cursor="hand2")
        self.btn_hw.pack(side=tk.LEFT, padx=2)

        # 2. STATUS TILES ROW (Compact)
        tiles_frame = tk.Frame(self, bg="#090d16", padx=10, pady=2)
        tiles_frame.pack(fill=tk.X)

        self.lbl_target = self.make_badge(tiles_frame, "TARGET (r)", "22.0 cm", "#38bdf8", 0)
        self.lbl_measured = self.make_badge(tiles_frame, "MEASURED (y)", "12.0 cm", "#10b981", 1)
        self.lbl_error = self.make_badge(tiles_frame, "ERROR (e)", "+10.0 cm", "#f43f5e", 2)
        self.lbl_servo = self.make_badge(tiles_frame, "SERVO (θ)", "90°", "#fbbf24", 3)
        self.lbl_vel = self.make_badge(tiles_frame, "VELOCITY (v)", "0.0 cm/s", "#2dd4bf", 4)
        self.lbl_iae = self.make_badge(tiles_frame, "ERROR (IAE)", "0.0", "#c084fc", 5)

        # 3. CENTER DISPLAY: HARDWARE RIG (TOP) + OSCILLOSCOPE (BOTTOM)
        center_frame = tk.Frame(self, bg="#090d16", padx=10)
        center_frame.pack(fill=tk.BOTH, expand=True)

        # 3A. Physical Viewport (Compact: 155px height)
        anim_box = tk.Frame(center_frame, bg="#131c2e", highlightthickness=1, highlightbackground="#1e293b")
        anim_box.pack(fill=tk.X, pady=2)

        anim_hdr = tk.Frame(anim_box, bg="#131c2e", padx=8, pady=2)
        anim_hdr.pack(fill=tk.X)
        self.anim_title_lbl = tk.Label(anim_hdr, text="PHYSICAL MECHANICAL RIG & REAL-TIME FORCE VECTORS", fg="#38bdf8", bg="#131c2e", font=("Segoe UI", 8, "bold"))
        self.anim_title_lbl.pack(side=tk.LEFT)

        v_leg = tk.Frame(anim_hdr, bg="#131c2e")
        v_leg.pack(side=tk.RIGHT)
        tk.Label(v_leg, text="→ Velocity (v)", fg="#34d399", bg="#131c2e", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT, padx=5)
        tk.Label(v_leg, text="→ Gravity (a)", fg="#fbbf24", bg="#131c2e", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT, padx=5)
        tk.Label(v_leg, text="→ PID Thrust (F)", fg="#00f2fe", bg="#131c2e", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT, padx=5)

        self.anim_canvas = tk.Canvas(anim_box, height=155, bg="#060911", highlightthickness=0)
        self.anim_canvas.pack(fill=tk.X, expand=True)

        # 3B. Oscilloscope (Compact: 105px height)
        graph_box = tk.Frame(center_frame, bg="#131c2e", highlightthickness=1, highlightbackground="#1e293b")
        graph_box.pack(fill=tk.BOTH, expand=True, pady=2)

        graph_hdr = tk.Frame(graph_box, bg="#131c2e", padx=8, pady=2)
        graph_hdr.pack(fill=tk.X)
        tk.Label(graph_hdr, text="TIME-DOMAIN STEP & SINE RESPONSE (OSCILLOSCOPE)", fg="#10b981", bg="#131c2e", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)
        tk.Label(graph_hdr, text="■ Target r(t)", fg="#38bdf8", bg="#131c2e", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=10)
        tk.Label(graph_hdr, text="■ Measured y(t)", fg="#10b981", bg="#131c2e", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=6)

        self.status_msg = tk.Label(graph_hdr, text="Active: Dynamic Sine Tracking", fg="#38bdf8", bg="#131c2e", font=("Segoe UI", 8))
        self.status_msg.pack(side=tk.RIGHT, padx=6)

        self.graph_canvas = tk.Canvas(graph_box, height=105, bg="#04060c", highlightthickness=0)
        self.graph_canvas.pack(fill=tk.BOTH, expand=True)

        # 4. BOTTOM CONTROLS (Compact & Fully Visible on all screens!)
        ctrl_frame = tk.Frame(self, bg="#131c2e", padx=10, pady=6, highlightthickness=1, highlightbackground="#1e293b")
        ctrl_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=(2, 6))

        sliders_box = tk.Frame(ctrl_frame, bg="#131c2e")
        sliders_box.pack(fill=tk.X)

        self.s_kp = self.add_slider(sliders_box, "Proportional (Kp):", 0.0, 8.0, self.kp, 0.1, "#38bdf8", 0)
        self.s_ki = self.add_slider(sliders_box, "Integral (Ki):", 0.0, 0.2, self.ki, 0.01, "#10b981", 1)
        self.s_kd = self.add_slider(sliders_box, "Derivative (Kd):", 0.0, 2.5, self.kd, 0.05, "#fbbf24", 2)
        self.s_target = self.add_slider(sliders_box, "Target Distance (cm):", 5.0, 45.0, self.target, 1.0, "#c084fc", 3)

        btn_row = tk.Frame(ctrl_frame, bg="#131c2e", pady=3)
        btn_row.pack(fill=tk.X)

        tk.Label(btn_row, text="Quick Set:", fg="#94a3b8", bg="#131c2e", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT, padx=2)
        for t_val in [15, 20, 30]:
            tk.Button(btn_row, text=f"{t_val} cm", command=lambda v=t_val: self.set_target(v), bg="#1e293b", fg="#38bdf8", relief="flat", padx=7, pady=1, font=("Segoe UI", 8, "bold"), cursor="hand2").pack(side=tk.LEFT, padx=2)

        tk.Label(btn_row, text="|", fg="#334155", bg="#131c2e").pack(side=tk.LEFT, padx=5)

        tk.Button(btn_row, text="⚡ Nudge Ball (+10 cm)", command=self.apply_nudge, bg="#e11d48", fg="white", relief="flat", padx=9, pady=2, font=("Segoe UI", 8, "bold"), cursor="hand2").pack(side=tk.LEFT, padx=3)
        tk.Button(btn_row, text="🌊 Toggle Sine / Step", command=self.toggle_trajectory, bg="#0284c7", fg="white", relief="flat", padx=9, pady=2, font=("Segoe UI", 8, "bold"), cursor="hand2").pack(side=tk.LEFT, padx=3)
        tk.Button(btn_row, text="Reset IAE", command=self.reset_metrics, bg="#1e293b", fg="#94a3b8", relief="flat", padx=7, pady=2, font=("Segoe UI", 7), cursor="hand2").pack(side=tk.LEFT, padx=3)

    def make_badge(self, parent, title, val, color, col):
        card = tk.Frame(parent, bg="#131c2e", padx=8, pady=4, highlightthickness=1, highlightbackground="#1e293b")
        card.grid(row=0, column=col, padx=2, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        tk.Label(card, text=title, fg="#64748b", bg="#131c2e", font=("Segoe UI", 7, "bold")).pack(anchor="w")
        lbl = tk.Label(card, text=val, fg=color, bg="#131c2e", font=("Consolas", 10, "bold"))
        lbl.pack(anchor="w", pady=1)
        return lbl

    def add_slider(self, parent, label, from_val, to_val, default_val, res, color, col):
        box = tk.Frame(parent, bg="#131c2e", padx=5)
        box.grid(row=0, column=col, sticky="ew")
        parent.grid_columnconfigure(col, weight=1)

        var = tk.DoubleVar(value=default_val)
        top = tk.Frame(box, bg="#131c2e")
        top.pack(fill=tk.X)
        tk.Label(top, text=label, fg="#cbd5e1", bg="#131c2e", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT)
        val_lbl = tk.Label(top, text=f"{default_val:.2f}", fg=color, bg="#131c2e", font=("Consolas", 8, "bold"))
        val_lbl.pack(side=tk.RIGHT)

        def on_change(v):
            val_lbl.config(text=f"{float(v):.2f}")
            self.on_param_update()

        scale = tk.Scale(box, from_=from_val, to=to_val, resolution=res, variable=var, orient=tk.HORIZONTAL, showvalue=False, command=on_change, bg="#1e293b", fg=color, highlightthickness=0, troughcolor="#060911", activebackground=color, cursor="hand2")
        scale.pack(fill=tk.X, pady=1)
        scale.var = var
        return scale

    def toggle_rig(self):
        if self.rig_type == "beam":
            self.rig_type = "tracker"
            self.btn_rig.config(text="Rig: 🎯 1-DOF Tracker", bg="#0284c7")
        else:
            self.rig_type = "beam"
            self.btn_rig.config(text="Rig: ⚖️ Ball-on-Beam", bg="#4f46e5")

    def toggle_controller_mode(self):
        if self.controller_type == "PID":
            self.controller_type = "ON_OFF"
            self.btn_mode.config(text="Mode: Bang-Bang", bg="#e11d48")
            self.status_msg.config(text="Mode: ON/OFF (Limit Cycles)", fg="#f43f5e")
        else:
            self.controller_type = "PID"
            self.btn_mode.config(text="Mode: PID", bg="#0284c7")
            self.status_msg.config(text="Mode: Closed-Loop PID", fg="#10b981")

    def toggle_trajectory(self):
        if self.trajectory_mode == "step":
            self.trajectory_mode = "sine"
            self.btn_traj.config(text="Trajectory: 🌊 Sine", bg="#0284c7")
            self.status_msg.config(text="Tracking: Sine Wave (0.20 Hz)", fg="#38bdf8")
        else:
            self.trajectory_mode = "step"
            self.btn_traj.config(text="Trajectory: Step", bg="#059669")
            self.status_msg.config(text="Tracking: Stationary Setpoint", fg="#10b981")

    def set_target(self, val):
        self.trajectory_mode = "step"
        self.btn_traj.config(text="Trajectory: Step", bg="#059669")
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

    def apply_nudge(self):
        self.position = min(46.0, self.position + 10.0)
        self.velocity += 14.0
        self.status_msg.config(text="Disturbance Injected (+10 cm)!", fg="#f43f5e")

    def reset_metrics(self):
        self.iae = 0.0

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
            self.btn_hw.config(text="USB", bg="#1e293b")
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

        # Ultrasonic Noise & Filter
        noise = random.gauss(0, self.noise_amp)
        raw_dist = max(2.0, min(50.0, self.position + noise))
        self.smoothed_dist = (self.alpha_dist * raw_dist) + ((1.0 - self.alpha_dist) * self.smoothed_dist)
        error = self.target - self.smoothed_dist
        self.iae += abs(error) * dt

        # Control Algorithm
        if self.controller_type == "ON_OFF":
            if error > 0.8:
                self.control_output = 28.0
            elif error < -0.8:
                self.control_output = -28.0
            else:
                self.control_output = 0.0
            self.servo_angle = 90.0 + self.control_output
        else:
            ctrl_error = 0.0 if abs(error) < 0.4 else error
            P = self.kp * ctrl_error

            self.integral += ctrl_error * dt
            self.integral = max(-35.0, min(35.0, self.integral))
            I = self.ki * self.integral

            raw_d = (ctrl_error - self.prev_error) / dt if dt > 0 else 0.0
            self.filtered_deriv = (self.alpha_d * raw_d) + ((1.0 - self.alpha_d) * self.filtered_deriv)
            D = self.kd * self.filtered_deriv
            self.prev_error = ctrl_error

            self.control_output = P + I + D
            self.servo_angle = max(25.0, min(155.0, 90.0 + self.control_output))

        # Dynamics
        if self.rig_type == "beam":
            self.beam_angle = (self.servo_angle - 90.0) * 0.35 # max +/- 22 deg
            g = 980.0 # cm/s^2
            theta_rad = math.radians(self.beam_angle)
            self.acceleration = (5.0 / 7.0) * g * math.sin(theta_rad) - (self.plant_damping * self.velocity)
            self.velocity += self.acceleration * dt
            self.position += self.velocity * dt
            self.ball_rotation = (self.ball_rotation + (self.velocity * dt / 0.15)) % 360.0
        else:
            tilt_rad = math.radians(self.servo_angle - 90.0)
            net_force = 120.0 * math.sin(tilt_rad) - (self.plant_damping * self.velocity)
            self.acceleration = net_force / self.plant_mass
            self.velocity += self.acceleration * dt
            self.position += self.velocity * dt

        # Boundaries
        if self.position <= 3.5:
            self.position = 3.5
            self.velocity = -0.3 * self.velocity
        elif self.position >= 47.5:
            self.position = 47.5
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
        pivot_x = w // 2 - 40
        pivot_y = h - 45
        beam_len = w - 240

        # Fulcrum
        c.create_polygon(pivot_x - 18, pivot_y + 28, pivot_x + 18, pivot_y + 28, pivot_x, pivot_y, fill="#1e293b", outline="#475569", width=2)
        c.create_oval(pivot_x - 5, pivot_y - 5, pivot_x + 5, pivot_y + 5, fill="#94a3b8")

        # Beam Geometry
        ang_rad = math.radians(self.beam_angle)
        half_len = beam_len / 2.0
        x1 = pivot_x - half_len * math.cos(ang_rad)
        y1 = pivot_y + half_len * math.sin(ang_rad)
        x2 = pivot_x + half_len * math.cos(ang_rad)
        y2 = pivot_y - half_len * math.sin(ang_rad)

        # Beam
        c.create_line(x1, y1 + 3, x2, y2 + 3, fill="#030712", width=6)
        c.create_line(x1, y1, x2, y2, fill="#0284c7", width=5)
        c.create_line(x1, y1, x2, y2, fill="#38bdf8", width=2)

        # HC-SR04 on Left Beam Tip
        c.create_rectangle(x1 - 10, y1 - 15, x1 + 10, y1 + 15, fill="#0284c7", outline="#38bdf8", width=2)
        c.create_oval(x1 + 4, y1 - 12, x1 + 12, y1 - 4, fill="#cbd5e1")
        c.create_oval(x1 + 4, y1 + 4, x1 + 12, y1 + 12, fill="#cbd5e1")
        c.create_text(x1 - 4, y1 - 22, text="HC-SR04", fill="#38bdf8", font=("Segoe UI", 7, "bold"))

        # Rolling Ball (Controlled Plant)
        pos_frac = (self.position - 3.5) / 44.0
        ball_x = x1 + pos_frac * (x2 - x1)
        ball_y = y1 + pos_frac * (y2 - y1) - 10

        # Ball Shadow
        c.create_oval(ball_x - 7, ball_y + 6, ball_x + 7, ball_y + 11, fill="#030712", outline="")

        # 3D Shaded Metallic Ball
        c.create_oval(ball_x - 9, ball_y - 9, ball_x + 9, ball_y + 9, fill="#f59e0b", outline="#fbbf24", width=2)
        c.create_oval(ball_x - 6, ball_y - 6, ball_x - 2, ball_y - 2, fill="#ffffff", outline="")

        # Rotating Spokes
        rot_rad = math.radians(self.ball_rotation)
        dx = 7 * math.cos(rot_rad)
        dy = 7 * math.sin(rot_rad)
        c.create_line(ball_x - dx, ball_y - dy, ball_x + dx, ball_y + dy, fill="#78350f", width=2)
        c.create_line(ball_x + dy, ball_y - dx, ball_x - dy, ball_y + dx, fill="#78350f", width=2)

        # Ball Label: Placed BELOW the beam to completely prevent text collision!
        c.create_text(ball_x, ball_y + 20, text=f"{self.position:.1f} cm", fill="#fbbf24", font=("Consolas", 8, "bold"))

        # Target Setpoint Flag: Placed strictly ABOVE the beam!
        tgt_frac = (self.target - 3.5) / 44.0
        tx = x1 + tgt_frac * (x2 - x1)
        ty = y1 + tgt_frac * (y2 - y1)
        c.create_line(tx, ty - 26, tx, ty, fill="#38bdf8", width=2, dash=(2, 2))
        c.create_polygon(tx, ty - 26, tx + 10, ty - 21, tx, ty - 16, fill="#38bdf8")
        c.create_text(tx, ty - 32, text=f"Target: {self.target:.0f}cm", fill="#38bdf8", font=("Segoe UI", 7, "bold"))

        # Sound Waves
        self.wave_phase = (self.wave_phase + 0.18) % 1.0
        gap_span = (ball_x - x1)
        if gap_span > 20:
            for arc_i in range(3):
                prog = (self.wave_phase + arc_i * 0.33) % 1.0
                wx = x1 + gap_span * prog
                wy = y1 + (ball_y - y1) * prog
                c.create_arc(wx - 7, wy - 7, wx + 7, wy + 7, start=-40, extent=80, style=tk.ARC, outline="#38bdf8", width=1.5)

        # Dynamic Real-Time Physics Vectors
        if self.show_vectors:
            # 1. Velocity (Green arrow)
            v_len = max(-40.0, min(40.0, self.velocity * 2.2))
            if abs(v_len) > 1.5:
                vx = ball_x + v_len * math.cos(ang_rad)
                vy = ball_y - v_len * math.sin(ang_rad)
                c.create_line(ball_x, ball_y, vx, vy, fill="#34d399", width=2.5, arrow=tk.LAST)
                c.create_text(vx, vy - 8, text="v", fill="#34d399", font=("Segoe UI", 7, "bold"))

            # 2. Gravity Acceleration (Yellow arrow)
            acc_len = max(-35.0, min(35.0, self.acceleration * 0.22))
            if abs(acc_len) > 1.5:
                ax = ball_x + acc_len * math.cos(ang_rad)
                ay = ball_y - acc_len * math.sin(ang_rad) - 14
                c.create_line(ball_x, ball_y - 14, ax, ay, fill="#fbbf24", width=2, arrow=tk.LAST)
                c.create_text(ax, ay - 6, text="a_grav", fill="#fbbf24", font=("Segoe UI", 6, "bold"))

            # 3. PID Thrust (Cyan arrow)
            f_len = max(-35.0, min(35.0, self.control_output * 1.3))
            if abs(f_len) > 1.5:
                fx = ball_x - f_len * math.cos(ang_rad)
                fy = ball_y + f_len * math.sin(ang_rad)
                c.create_line(ball_x, ball_y, fx, fy, fill="#00f2fe", width=2, arrow=tk.LAST)

        # Servo Motor on Right
        servo_x = w - 75
        servo_y = h - 45
        c.create_rectangle(servo_x - 18, servo_y - 18, servo_x + 18, servo_y + 18, fill="#0284c7", outline="#38bdf8", width=1.5)
        c.create_oval(servo_x - 5, servo_y - 5, servo_x + 5, servo_y + 5, fill="#ffffff")

        horn_rad = math.radians(self.servo_angle - 180)
        hx = servo_x + 20 * math.cos(horn_rad)
        hy = servo_y + 20 * math.sin(horn_rad)
        c.create_line(servo_x, servo_y, hx, hy, fill="#fbbf24", width=3)
        c.create_line(hx, hy, x2, y2, fill="#94a3b8", width=1.5, dash=(3, 2))
        c.create_text(servo_x, servo_y + 24, text=f"Servo: {int(self.servo_angle)}°", fill="#fbbf24", font=("Consolas", 7, "bold"))

    def draw_linear_tracker(self, c, w, h):
        track_y = h - 35
        start_x = 90
        end_x = w - 160
        track_w = end_x - start_x

        c.create_line(start_x - 10, track_y, end_x + 10, track_y, fill="#334155", width=2)
        for cm in range(0, 55, 10):
            tx = start_x + (cm / 50.0) * track_w
            c.create_line(tx, track_y, tx, track_y + 5, fill="#64748b", width=1)
            c.create_text(tx, track_y + 14, text=f"{cm}cm", fill="#94a3b8", font=("Consolas", 7))

        sensor_x = start_x - 45
        sensor_y = track_y - 20
        c.create_rectangle(sensor_x - 16, sensor_y - 18, sensor_x + 16, sensor_y + 18, fill="#0284c7", outline="#38bdf8", width=2)
        c.create_oval(sensor_x - 10, sensor_y - 8, sensor_x, sensor_y - 2, fill="#cbd5e1")
        c.create_oval(sensor_x - 10, sensor_y + 2, sensor_x, sensor_y + 8, fill="#cbd5e1")
        c.create_text(sensor_x, sensor_y - 26, text="HC-SR04", fill="#38bdf8", font=("Segoe UI", 7, "bold"))

        tgt_clamped = max(0.0, min(50.0, self.target))
        tgt_x = start_x + (tgt_clamped / 50.0) * track_w
        c.create_line(tgt_x, track_y - 40, tgt_x, track_y, fill="#38bdf8", width=2, dash=(3, 2))
        c.create_polygon(tgt_x, track_y - 40, tgt_x + 10, track_y - 34, tgt_x, track_y - 28, fill="#38bdf8")

        pos_clamped = max(0.0, min(50.0, self.position))
        obj_x = start_x + (pos_clamped / 50.0) * track_w
        obj_y = track_y - 15
        c.create_rectangle(obj_x - 5, obj_y - 24, obj_x + 5, obj_y + 15, fill="#e2e8f0", outline="#38bdf8", width=2)
        c.create_text(obj_x, obj_y - 30, text=f"{self.position:.1f} cm", fill="#10b981", font=("Consolas", 7, "bold"))

        # Vectors for Linear Tracker
        if self.show_vectors:
            v_len = max(-45.0, min(45.0, self.velocity * 2.5))
            if abs(v_len) > 2.0:
                c.create_line(obj_x, obj_y, obj_x + v_len, obj_y, fill="#34d399", width=2.5, arrow=tk.LAST)
                c.create_text(obj_x + v_len, obj_y - 10, text="v", fill="#34d399", font=("Segoe UI", 7, "bold"))

        # Sonar pulses
        self.wave_phase = (self.wave_phase + 0.18) % 1.0
        gap = (obj_x - 5) - (sensor_x + 16)
        if gap > 15:
            for w_idx in range(3):
                prog = (self.wave_phase + w_idx * 0.33) % 1.0
                wx = (sensor_x + 16) + (gap * prog)
                r = 8 + 10 * prog
                c.create_arc(wx - r, sensor_y - r, wx + r, sensor_y + r, start=-40, extent=80, style=tk.ARC, outline="#38bdf8", width=1.5)

        # Virtual 16x2 LCD
        lcd_x = w - 80
        lcd_y = track_y - 20
        c.create_rectangle(lcd_x - 40, lcd_y - 16, lcd_x + 40, lcd_y + 18, fill="#1e3a8a", outline="#60a5fa", width=1.5)
        err = self.target - self.smoothed_dist
        e_sign = "+" if err > 0 else ""
        c.create_text(lcd_x - 34, lcd_y - 7, text=f"T:{self.target:.0f} D:{self.smoothed_dist:.1f}", fill="#ffffff", font=("Courier", 7, "bold"), anchor="w")
        c.create_text(lcd_x - 34, lcd_y + 7, text=f"E:{e_sign}{err:.1f} S:{int(self.servo_angle)}", fill="#ffffff", font=("Courier", 7, "bold"), anchor="w")

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
            c.create_text(22, y_px - 6, text=f"{cm}cm", fill="#64748b", font=("Consolas", 7))

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
        self.lbl_vel.config(text=f"{self.velocity:+.1f} cm/s")
        self.lbl_iae.config(text=f"{self.iae:.1f}")

        self.draw_mechanical_rig()
        self.draw_graph()

        self.after(28, self.run_loop)


if __name__ == "__main__":
    app = VisualPIDSimulatorApp()
    app.mainloop()
