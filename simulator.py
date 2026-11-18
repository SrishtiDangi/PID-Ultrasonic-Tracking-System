"""
PID Ultrasonic Tracking & Balancing System
Clean, Intuitive Simulation & Telemetry Bench
Directly reflects the physical hardware: HC-SR04 Sensor + Servo Motor + 16x2 LCD
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


class CleanPIDSimulator(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PID Ultrasonic Tracking System - Simulation & Control")
        # Designed to fit comfortably on all standard screens (720p to 1080p)
        self.geometry("1000x700")
        self.configure(bg="#12161f")
        self.minsize(920, 640)

        # Operational States
        self.mode = "sim"                     # "sim" or "hw"
        self.controller_type = "PID"          # "PID" or "ON_OFF"
        self.is_hw_connected = False
        self.ser = None

        # Plant / Physical State
        self.target = 20.0                    # cm (desired distance)
        self.position = 20.0                  # cm (actual object position)
        self.velocity = 0.0                  # cm/s
        self.plant_mass = 1.0                # kg
        self.plant_damping = 2.0             # damping friction
        self.noise_amp = 0.25                # sensor acoustic jitter (cm)

        # PID Parameters (matching Arduino firmware)
        self.kp = 2.0
        self.ki = 0.02
        self.kd = 0.10
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_deriv = 0.0
        self.alpha_d = 0.20
        self.alpha_dist = 0.30
        self.smoothed_dist = 20.0
        self.control_output = 0.0
        self.servo_angle = 90.0

        # Oscilloscope Buffers (120 points)
        self.buf_len = 120
        self.target_buf = [20.0] * self.buf_len
        self.pos_buf = [20.0] * self.buf_len

        # Time & animation
        self.last_time = time.time()
        self.wave_phase = 0.0

        self.build_ui()
        self.run_loop()

    def build_ui(self):
        # 1. HEADER BAR
        header = tk.Frame(self, bg="#1a202c", padx=16, pady=10)
        header.pack(fill=tk.X, side=tk.TOP)

        title_box = tk.Frame(header, bg="#1a202c")
        title_box.pack(side=tk.LEFT)
        tk.Label(title_box, text="PID Ultrasonic Distance Tracking System", fg="#38bdf8", bg="#1a202c", font=("Segoe UI", 13, "bold")).pack(anchor="w")
        tk.Label(title_box, text="Hardware-Software Simulation & Telemetry Bench", fg="#94a3b8", bg="#1a202c", font=("Segoe UI", 9)).pack(anchor="w")

        controls_box = tk.Frame(header, bg="#1a202c")
        controls_box.pack(side=tk.RIGHT)

        # PID vs Bang-Bang Toggle
        self.btn_algo = tk.Button(controls_box, text="Controller: PID", command=self.toggle_algo, bg="#0284c7", fg="white", font=("Segoe UI", 9, "bold"), relief="flat", padx=10, pady=3, cursor="hand2")
        self.btn_algo.pack(side=tk.LEFT, padx=6)

        # Source Toggle: Simulation vs Hardware
        self.btn_source = tk.Button(controls_box, text="Source: Simulation", command=self.toggle_source, bg="#334155", fg="#cbd5e1", font=("Segoe UI", 9, "bold"), relief="flat", padx=10, pady=3, cursor="hand2")
        self.btn_source.pack(side=tk.LEFT, padx=6)

        self.port_combo = ttk.Combobox(controls_box, width=8, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)
        self.refresh_ports()

        self.btn_hw = tk.Button(controls_box, text="Connect", command=self.toggle_serial, bg="#10b981", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=8, pady=3, cursor="hand2")
        self.btn_hw.pack(side=tk.LEFT, padx=3)

        # 2. STATUS TILES ROW (Clean & Direct)
        tiles_frame = tk.Frame(self, bg="#12161f", padx=14, pady=8)
        tiles_frame.pack(fill=tk.X)

        self.lbl_target = self.make_tile(tiles_frame, "TARGET DISTANCE", "20.0 cm", "#38bdf8", 0)
        self.lbl_measured = self.make_tile(tiles_frame, "MEASURED DISTANCE", "20.0 cm", "#10b981", 1)
        self.lbl_error = self.make_tile(tiles_frame, "ERROR (Target - Dist)", "0.0 cm", "#f43f5e", 2)
        self.lbl_servo = self.make_tile(tiles_frame, "SERVO ANGLE", "90°", "#fbbf24", 3)

        # 3. MAIN CENTER SECTION: HARDWARE ANIMATION (TOP) + GRAPH (BOTTOM)
        center_frame = tk.Frame(self, bg="#12161f", padx=14)
        center_frame.pack(fill=tk.BOTH, expand=True)

        # 3A. PHYSICAL HARDWARE VIEW (Clear HC-SR04 + Sound Waves + Moving Object + Servo + LCD)
        anim_card = tk.LabelFrame(center_frame, text=" Physical Hardware Representation (HC-SR04 Sensor & Servo Tracking) ", fg="#38bdf8", bg="#1a202c", font=("Segoe UI", 9, "bold"), padx=10, pady=6)
        anim_card.pack(fill=tk.X, pady=4)

        self.anim_canvas = tk.Canvas(anim_card, height=140, bg="#0f172a", highlightthickness=1, highlightbackground="#334155")
        self.anim_canvas.pack(fill=tk.X, expand=True)

        # 3B. REAL-TIME OSCILLOSCOPE GRAPH (Target vs Distance)
        graph_card = tk.LabelFrame(center_frame, text=" Real-Time Telemetry Graph (Target Setpoint vs Measured Distance) ", fg="#10b981", bg="#1a202c", font=("Segoe UI", 9, "bold"), padx=10, pady=6)
        graph_card.pack(fill=tk.BOTH, expand=True, pady=4)

        # Legend row
        legend_row = tk.Frame(graph_card, bg="#1a202c")
        legend_row.pack(fill=tk.X, pady=2)
        tk.Label(legend_row, text="■ Target Distance (Cyan)", fg="#38bdf8", bg="#1a202c", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=10)
        tk.Label(legend_row, text="■ Measured Distance (Green)", fg="#10b981", bg="#1a202c", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=10)
        self.status_msg = tk.Label(legend_row, text="Status: Stable at Target", fg="#94a3b8", bg="#1a202c", font=("Segoe UI", 8))
        self.status_msg.pack(side=tk.RIGHT, padx=10)

        self.graph_canvas = tk.Canvas(graph_card, bg="#090d16", highlightthickness=1, highlightbackground="#334155")
        self.graph_canvas.pack(fill=tk.BOTH, expand=True)

        # 4. BOTTOM CONTROLS & PID TUNING SLIDERS
        ctrl_card = tk.LabelFrame(self, text=" Controls & Live PID Tuning (Directly adjusts system response) ", fg="#fbbf24", bg="#1a202c", font=("Segoe UI", 9, "bold"), padx=12, pady=6)
        ctrl_card.pack(fill=tk.X, side=tk.BOTTOM, padx=14, pady=8)

        # Sliders Row
        sliders_frame = tk.Frame(ctrl_card, bg="#1a202c")
        sliders_frame.pack(fill=tk.X)

        self.s_kp = self.add_slider(sliders_frame, "Kp (Proportional):", 0.0, 8.0, self.kp, 0.1, 0)
        self.s_ki = self.add_slider(sliders_frame, "Ki (Integral):", 0.0, 0.2, self.ki, 0.01, 1)
        self.s_kd = self.add_slider(sliders_frame, "Kd (Derivative):", 0.0, 2.0, self.kd, 0.05, 2)
        self.s_target = self.add_slider(sliders_frame, "Target Distance (cm):", 5.0, 45.0, self.target, 1.0, 3)

        # Quick Buttons Row
        btn_row = tk.Frame(ctrl_card, bg="#1a202c", pady=4)
        btn_row.pack(fill=tk.X)

        tk.Label(btn_row, text="Quick Target Steps:", fg="#94a3b8", bg="#1a202c", font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=4)
        for t_val in [15, 20, 30]:
            tk.Button(btn_row, text=f"Set {t_val} cm", command=lambda v=t_val: self.set_target(v), bg="#334155", fg="white", relief="flat", padx=8, pady=1, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=3)

        tk.Label(btn_row, text="|", fg="#475569", bg="#1a202c").pack(side=tk.LEFT, padx=8)

        # Disturbance Test Button (Easy to explain to professor)
        tk.Button(btn_row, text="⚡ Move Object by Hand (+10 cm)", command=self.apply_disturbance, bg="#e11d48", fg="white", relief="flat", padx=10, pady=2, font=("Segoe UI", 8, "bold"), cursor="hand2").pack(side=tk.LEFT, padx=4)
        tk.Button(btn_row, text="Reset to Center", command=self.reset_system, bg="#475569", fg="white", relief="flat", padx=8, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=4)

    def make_tile(self, parent, title, val, color, col):
        frame = tk.Frame(parent, bg="#1a202c", padx=12, pady=6, highlightthickness=1, highlightbackground="#334155")
        frame.grid(row=0, column=col, padx=4, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        tk.Label(frame, text=title, fg="#94a3b8", bg="#1a202c", font=("Segoe UI", 7, "bold")).pack(anchor="w")
        lbl = tk.Label(frame, text=val, fg=color, bg="#1a202c", font=("Segoe UI", 12, "bold"))
        lbl.pack(anchor="w", pady=1)
        return lbl

    def add_slider(self, parent, label, from_val, to_val, default_val, res, col):
        box = tk.Frame(parent, bg="#1a202c", padx=6)
        box.grid(row=0, column=col, sticky="ew")
        parent.grid_columnconfigure(col, weight=1)

        var = tk.DoubleVar(value=default_val)
        top = tk.Frame(box, bg="#1a202c")
        top.pack(fill=tk.X)
        tk.Label(top, text=label, fg="#cbd5e1", bg="#1a202c", font=("Segoe UI", 8)).pack(side=tk.LEFT)
        val_lbl = tk.Label(top, text=f"{default_val:.2f}", fg="#38bdf8", bg="#1a202c", font=("Segoe UI", 8, "bold"))
        val_lbl.pack(side=tk.RIGHT)

        def on_change(v):
            val_lbl.config(text=f"{float(v):.2f}")
            self.on_param_update()

        scale = tk.Scale(box, from_=from_val, to=to_val, resolution=res, variable=var, orient=tk.HORIZONTAL, showvalue=False, command=on_change, bg="#334155", fg="#38bdf8", highlightthickness=0, troughcolor="#0f172a", activebackground="#38bdf8")
        scale.pack(fill=tk.X, pady=1)
        scale.var = var
        return scale

    def set_target(self, val):
        self.target = float(val)
        self.s_target.var.set(val)
        self.on_param_update()

    def on_param_update(self):
        self.kp = self.s_kp.var.get()
        self.ki = self.s_ki.var.get()
        self.kd = self.s_kd.var.get()
        self.target = self.s_target.var.get()

        if self.mode == "hw" and self.is_hw_connected and self.ser:
            try:
                cmd = f"P{self.kp}\nI{self.ki}\nD{self.kd}\nT{self.target}\n"
                self.ser.write(cmd.encode())
            except Exception:
                pass

    def apply_disturbance(self):
        # Simulate pushing the object 10 cm away
        self.position = min(48.0, self.position + 10.0)
        self.velocity += 12.0
        self.status_msg.config(text="Disturbance Injected! PID Compensating...", fg="#f43f5e")

    def reset_system(self):
        self.position = self.target
        self.velocity = 0.0
        self.integral = 0.0
        self.status_msg.config(text="Status: Reset to Target", fg="#94a3b8")

    def toggle_algo(self):
        if self.controller_type == "PID":
            self.controller_type = "ON_OFF"
            self.btn_algo.config(text="Controller: ON/OFF (Bang-Bang)", bg="#e11d48")
            self.status_msg.config(text="ON/OFF Mode: Notice continuous hunting/oscillations!", fg="#f43f5e")
        else:
            self.controller_type = "PID"
            self.btn_algo.config(text="Controller: PID", bg="#0284c7")
            self.status_msg.config(text="PID Mode: Smooth exponential settling to zero error.", fg="#10b981")

    def toggle_source(self):
        if self.mode == "sim":
            self.mode = "hw"
            self.btn_source.config(text="Source: Hardware (USB)", bg="#0284c7", fg="white")
        else:
            self.mode = "sim"
            self.btn_source.config(text="Source: Simulation", bg="#334155", fg="#cbd5e1")

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
                self.btn_source.config(text="Source: Hardware (USB)", bg="#0284c7", fg="white")
                self.btn_hw.config(text="Disconnect", bg="#e11d48")
                threading.Thread(target=self.serial_reader, daemon=True).start()
            except Exception as e:
                messagebox.showerror("Error", str(e))
        else:
            self.is_hw_connected = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.btn_hw.config(text="Connect", bg="#10b981")
            self.mode = "sim"
            self.btn_source.config(text="Source: Simulation", bg="#334155", fg="#cbd5e1")

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
            return # Readings come from physical Arduino

        # 1. Ultrasonic Reading + Jitter
        noise = random.gauss(0, self.noise_amp)
        raw_dist = max(2.0, min(50.0, self.position + noise))

        # 2. EMA Filter (Matching Arduino code)
        self.smoothed_dist = (self.alpha_dist * raw_dist) + ((1.0 - self.alpha_dist) * self.smoothed_dist)
        error = self.target - self.smoothed_dist

        # 3. Control Algorithm: PID vs Bang-Bang
        if self.controller_type == "ON_OFF":
            if error > 0.8:
                self.control_output = 30.0
            elif error < -0.8:
                self.control_output = -30.0
            else:
                self.control_output = 0.0
            self.servo_angle = 90.0 + self.control_output
        else:
            # PID Controller
            ctrl_error = 0.0 if abs(error) < 0.6 else error
            P = self.kp * ctrl_error

            self.integral += ctrl_error * dt
            self.integral = max(-40.0, min(40.0, self.integral))
            I = self.ki * self.integral

            raw_d = (ctrl_error - self.prev_error) / dt if dt > 0 else 0.0
            self.filtered_deriv = (self.alpha_d * raw_d) + ((1.0 - self.alpha_d) * self.filtered_deriv)
            D = self.kd * self.filtered_deriv
            self.prev_error = ctrl_error

            self.control_output = P + I + D
            self.servo_angle = max(20.0, min(160.0, 90.0 + self.control_output))

        # 4. Actuator pushes the object
        tilt_rad = math.radians(self.servo_angle - 90.0)
        actuator_force = 120.0 * math.sin(tilt_rad)
        damping_force = -self.plant_damping * self.velocity

        net_force = actuator_force + damping_force
        accel = net_force / self.plant_mass

        self.velocity += accel * dt
        self.position += self.velocity * dt

        if self.position <= 3.0:
            self.position = 3.0
            self.velocity = 0.0
        elif self.position >= 48.0:
            self.position = 48.0
            self.velocity = 0.0

    def draw_hardware_view(self):
        """Draws the exact physical components: HC-SR04 Sensor + Sound Waves + Target Object + Servo + 16x2 LCD"""
        c = self.anim_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        track_y = h - 35
        start_x = 90
        end_x = w - 210  # Leave room on the right for Servo & 16x2 LCD
        track_w = end_x - start_x

        # 1. Distance Track Line with cm markings
        c.create_line(start_x - 10, track_y, end_x + 10, track_y, fill="#334155", width=2)
        for cm in range(0, 55, 10):
            tx = start_x + (cm / 50.0) * track_w
            c.create_line(tx, track_y, tx, track_y + 6, fill="#64748b", width=1)
            c.create_text(tx, track_y + 16, text=f"{cm}cm", fill="#94a3b8", font=("Segoe UI", 7))

        # 2. HC-SR04 Ultrasonic Sensor Graphic (Exact blue module with two silver eyes)
        sensor_x = start_x - 45
        sensor_y = track_y - 25
        # Blue PCB
        c.create_rectangle(sensor_x - 18, sensor_y - 22, sensor_x + 18, sensor_y + 22, fill="#0284c7", outline="#38bdf8", width=2)
        # Left Eye (Transmitter T)
        c.create_oval(sensor_x - 12, sensor_y - 18, sensor_x + 12, sensor_y - 2, fill="#cbd5e1", outline="#94a3b8", width=2)
        c.create_text(sensor_x, sensor_y - 10, text="T", fill="#0f172a", font=("Segoe UI", 7, "bold"))
        # Right Eye (Receiver R)
        c.create_oval(sensor_x - 12, sensor_y + 2, sensor_x + 12, sensor_y + 18, fill="#cbd5e1", outline="#94a3b8", width=2)
        c.create_text(sensor_x, sensor_y + 10, text="R", fill="#0f172a", font=("Segoe UI", 7, "bold"))
        c.create_text(sensor_x, sensor_y - 30, text="HC-SR04", fill="#38bdf8", font=("Segoe UI", 8, "bold"))

        # 3. Target Setpoint Marker (Cyan Flag)
        tgt_clamped = max(0.0, min(50.0, self.target))
        tgt_x = start_x + (tgt_clamped / 50.0) * track_w
        c.create_line(tgt_x, track_y - 50, tgt_x, track_y, fill="#38bdf8", width=2, dash=(3, 2))
        c.create_polygon(tgt_x, track_y - 50, tgt_x + 12, track_y - 42, tgt_x, track_y - 34, fill="#38bdf8")
        c.create_text(tgt_x, track_y - 58, text=f"Target: {self.target:.0f}cm", fill="#38bdf8", font=("Segoe UI", 8, "bold"))

        # 4. Moving Object / Obstacle (Physical tracked plate)
        pos_clamped = max(0.0, min(50.0, self.position))
        obj_x = start_x + (pos_clamped / 50.0) * track_w
        obj_y = track_y - 18
        # Flat reflective plate
        c.create_rectangle(obj_x - 4, obj_y - 25, obj_x + 4, obj_y + 18, fill="#e2e8f0", outline="#38bdf8", width=2)
        # Base cart
        c.create_rectangle(obj_x - 14, obj_y + 8, obj_x + 14, obj_y + 18, fill="#475569", outline="#64748b")
        c.create_text(obj_x, obj_y - 32, text=f"{self.position:.1f}cm", fill="#10b981", font=("Segoe UI", 8, "bold"))

        # 5. Animated Ultrasonic Sound Waves (Acoustic ping traveling from sensor to object)
        self.wave_phase = (self.wave_phase + 0.18) % 1.0
        gap = (obj_x - 4) - (sensor_x + 18)
        if gap > 20:
            for w_idx in range(3):
                prog = (self.wave_phase + w_idx * 0.33) % 1.0
                wx = (sensor_x + 18) + (gap * prog)
                r = 10 + 12 * prog
                c.create_arc(wx - r, sensor_y - r, wx + r, sensor_y + r, start=-40, extent=80, style=tk.ARC, outline="#38bdf8", width=2)

        # 6. Physical Servo Motor (SG90) Graphic
        servo_center_x = end_x + 45
        servo_center_y = track_y - 20
        # Translucent blue casing
        c.create_rectangle(servo_center_x - 18, servo_center_y - 18, servo_center_x + 18, servo_center_y + 18, fill="#0284c7", outline="#38bdf8", width=1)
        c.create_oval(servo_center_x - 6, servo_center_y - 6, servo_center_x + 6, servo_center_y + 6, fill="#f8fafc")
        # Rotating Horn
        horn_rad = math.radians(self.servo_angle - 180)
        horn_len = 24
        hx = servo_center_x + horn_len * math.cos(horn_rad)
        hy = servo_center_y + horn_len * math.sin(horn_rad)
        c.create_line(servo_center_x, servo_center_y, hx, hy, fill="#fbbf24", width=4)
        c.create_text(servo_center_x, servo_center_y + 26, text=f"Servo: {self.servo_angle:.0f}°", fill="#fbbf24", font=("Segoe UI", 8, "bold"))

        # 7. Live 16x2 LCD Display Mockup (Directly mirrors the physical screen!)
        lcd_x = end_x + 115
        lcd_y = track_y - 35
        # Blue LCD Bezel
        c.create_rectangle(lcd_x - 45, lcd_y - 20, lcd_x + 45, lcd_y + 24, fill="#1e3a8a", outline="#60a5fa", width=2)
        # LCD Screen Area (Bright blue with white text)
        c.create_rectangle(lcd_x - 41, lcd_y - 16, lcd_x + 41, lcd_y + 20, fill="#0284c7", outline="")
        err = self.target - self.smoothed_dist
        e_sign = "+" if err > 0 else ""
        # Line 1: T:20.0 D:20.1
        c.create_text(lcd_x - 38, lcd_y - 8, text=f"T:{self.target:.0f} D:{self.smoothed_dist:.1f}", fill="#ffffff", font=("Courier", 7, "bold"), anchor="w")
        # Line 2: E:+0.1 S:90
        c.create_text(lcd_x - 38, lcd_y + 8, text=f"E:{e_sign}{err:.1f} S:{int(self.servo_angle)}", fill="#ffffff", font=("Courier", 7, "bold"), anchor="w")
        c.create_text(lcd_x, lcd_y - 26, text="16x2 LCD Display", fill="#94a3b8", font=("Segoe UI", 7))

    def draw_graph(self):
        """Draws clean, unambiguous time-domain graph: Target vs Measured Distance"""
        c = self.graph_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        y_max = 50.0  # cm

        # Horizontal Grid Lines
        for cm in [10, 20, 30, 40]:
            y_px = h - int((cm / y_max) * h)
            c.create_line(0, y_px, w, y_px, fill="#1e293b", dash=(2, 4))
            c.create_text(24, y_px - 8, text=f"{cm} cm", fill="#64748b", font=("Segoe UI", 7))

        dx = w / (self.buf_len - 1)

        # 1. Target Line (Cyan)
        target_pts = []
        for i, val in enumerate(self.target_buf):
            x = i * dx
            y = h - (val / y_max) * h
            target_pts.extend([x, max(0, min(h, y))])
        if len(target_pts) >= 4:
            c.create_line(*target_pts, fill="#38bdf8", width=2)

        # 2. Measured Distance Line (Green for PID, Red for Bang-Bang)
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

        # Update buffers
        self.target_buf.pop(0)
        self.target_buf.append(self.target)

        self.pos_buf.pop(0)
        self.pos_buf.append(self.position)

        # Update labels
        self.lbl_target.config(text=f"{self.target:.1f} cm")
        self.lbl_measured.config(text=f"{self.position:.1f} cm")
        err = self.target - self.position
        sign = "+" if err > 0 else ""
        self.lbl_error.config(text=f"{sign}{err:.1f} cm")
        self.lbl_servo.config(text=f"{int(self.servo_angle)}°")

        self.draw_hardware_view()
        self.draw_graph()

        self.after(30, self.run_loop)


if __name__ == "__main__":
    app = CleanPIDSimulator()
    app.mainloop()
