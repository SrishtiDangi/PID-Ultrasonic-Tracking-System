"""
PID Closed-Loop Ultrasonic Tracking & Balancing System
Ultra-Interactive Multi-Environment Physics Simulator & Control Bench
Evaluation: Control Systems / Embedded Systems Laboratory (100 Marks)

Core Innovations:
1. Multi-Environment Visualizer:
   - 🌿 Greenhouse / Plant Canopy Height & Irrigation Tracker (Actual Botanical Plants!)
   - 🚗 Autonomous Vehicle / Adaptive Cruise Control (ACC)
   - 🔬 Laboratory Hardware Bench Apparatus (HC-SR04 & Micro Servo)
2. Live Parameter Impact Engine:
   - Instant visual feedback when moving Kp, Ki, Kd, Target, or Noise
   - Live P, I, D Action Breakdown Bars (shows exact contribution of each term)
   - Real-time Physics Force Vectors (PID Drive Force vs Damping Drag)
   - Automatic micro-settling response so sliders visibly change plant behavior immediately
   - Educational insight ticker explaining the control theory of what just changed
3. Bang-Bang vs PID Comparative Analysis & System Power Toggle
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


class PIDPhysicsSimulator(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AERO-PID // Advanced Closed-Loop Autonomous Control Bench (100 Marks)")
        self.geometry("1140x860")
        self.configure(bg="#080a0f")
        self.minsize(1000, 760)

        # Operational States
        self.mode = "sim"
        self.system_enabled = True
        self.controller_type = "PID"
        self.env_view = "plant"               # "plant" (Botanical), "vehicle" (Car ACC), "bench" (Lab Track)
        self.ser = None
        self.is_hw_connected = False

        # Physical Plant Dynamics (2nd-order system: mass, damper, spring)
        self.position = 20.0                  # cm (or meters in vehicle mode)
        self.velocity = 0.0                  # cm/s
        self.target = 20.0                    # cm
        self.plant_mass = 1.0                # kg
        self.plant_damping = 1.8             # N*s/cm
        self.noise_amplitude = 0.35           # cm

        # PID Controller Parameters & Decomposition
        self.kp = 2.4
        self.ki = 0.05
        self.kd = 0.85
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.alpha_deriv = 0.22
        self.alpha_ema = 0.30
        self.smoothed_dist = 20.0
        self.raw_dist = 20.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.control_output = 0.0
        self.servo_angle = 90.0

        # Explanation Ticker
        self.insight_text = "System Ready. Adjust sliders or click step buttons to observe dynamic PID compensation."
        self.insight_color = "#38bdf8"

        # Transient Response Analytics
        self.step_start_time = time.time()
        self.step_initial_pos = 20.0
        self.rise_time = 0.0
        self.settling_time = 0.0
        self.peak_overshoot = 0.0
        self.steady_state_error = 0.0
        self.step_monitoring = True

        # Oscilloscope Buffers (140 points)
        self.buffer_len = 140
        self.target_buffer = [20.0] * self.buffer_len
        self.pos_buffer = [20.0] * self.buffer_len
        self.ctrl_buffer = [0.0] * self.buffer_len

        # Visual Loop Variables
        self.last_sim_time = time.time()
        self.wave_phase = 0.0
        self.road_scroll = 0.0
        self.plant_sway = 0.0
        self.spray_phase = 0.0

        self.build_ui()
        self.animate_loop()

    def build_ui(self):
        # 1. TOP HEADER / COCKPIT STATUS BAR
        header = tk.Frame(self, bg="#0d111a", padx=14, pady=8, highlightthickness=1, highlightbackground="#1e293b")
        header.pack(fill=tk.X, side=tk.TOP)

        title_frame = tk.Frame(header, bg="#0d111a")
        title_frame.pack(side=tk.LEFT)

        h_title = tk.Frame(title_frame, bg="#0d111a")
        h_title.pack(anchor="w")
        tk.Label(h_title, text="CONTROL-X", fg="#00f2fe", bg="#0d111a", font=("Segoe UI", 13, "bold")).pack(side=tk.LEFT)
        tk.Label(h_title, text=" // DUAL DOMAIN PID & PHYSICAL PLANT BENCH", fg="#ffffff", bg="#0d111a", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)

        self.sub_header_lbl = tk.Label(title_frame, text="Active Context: 🌿 Smart Agriculture / Botanical Canopy Distance Keeping", fg="#10b981", bg="#0d111a", font=("Segoe UI", 8))
        self.sub_header_lbl.pack(anchor="w", pady=1)

        # Switchboard (Top Right)
        switchboard = tk.Frame(header, bg="#0d111a")
        switchboard.pack(side=tk.RIGHT)

        # Power Toggle
        self.btn_power = tk.Button(switchboard, text="● SYSTEM ACTIVE", command=self.toggle_system_power, bg="#059669", fg="#ffffff", font=("Segoe UI", 8, "bold"), relief="flat", padx=10, pady=3, activebackground="#10b981", cursor="hand2")
        self.btn_power.pack(side=tk.LEFT, padx=4)

        # Algorithm Selector
        self.btn_algo = tk.Button(switchboard, text="MODE: PID CONTROLLER", command=self.toggle_controller_algo, bg="#1e293b", fg="#38bdf8", font=("Segoe UI", 8, "bold"), relief="flat", padx=10, pady=3, activebackground="#334155", cursor="hand2")
        self.btn_algo.pack(side=tk.LEFT, padx=4)

        # 3-Way Environment Switcher
        self.btn_env = tk.Button(switchboard, text="ENV: 🌿 GREENHOUSE PLANTS", command=self.cycle_env_view, bg="#047857", fg="#ffffff", font=("Segoe UI", 8, "bold"), relief="flat", padx=10, pady=3, activebackground="#059669", cursor="hand2")
        self.btn_env.pack(side=tk.LEFT, padx=4)

        # USB Connection
        self.port_combo = ttk.Combobox(switchboard, width=7, state="readonly", font=("Segoe UI", 8))
        self.port_combo.pack(side=tk.LEFT, padx=3)
        self.refresh_ports()
        self.btn_hw_connect = tk.Button(switchboard, text="USB", command=self.toggle_hw_connection, bg="#1e293b", fg="#94a3b8", font=("Segoe UI", 8, "bold"), relief="flat", padx=6, pady=3, cursor="hand2")
        self.btn_hw_connect.pack(side=tk.LEFT, padx=2)

        # 2. TELEMETRY BADGES ROW
        cards_bar = tk.Frame(self, bg="#080a0f", padx=10, pady=5)
        cards_bar.pack(fill=tk.X)

        self.card_target = self.create_badge(cards_bar, "TARGET SETPOINT (r)", "20.0 cm", "#00f2fe", 0)
        self.card_pos = self.create_badge(cards_bar, "PLANT DISTANCE (y)", "20.0 cm", "#38bdf8", 1)
        self.card_error = self.create_badge(cards_bar, "TRACKING ERROR (e)", "0.0 cm", "#f43f5e", 2)
        self.card_mode = self.create_badge(cards_bar, "ALGORITHM STATE", "PID CONVERGED", "#10b981", 3)
        self.card_settling = self.create_badge(cards_bar, "SETTLING TIME (ts)", "1.75 s", "#fbbf24", 4)
        self.card_overshoot = self.create_badge(cards_bar, "PEAK OVERSHOOT (Mp)", "7.8 %", "#a78bfa", 5)

        # 3. LIVE "WHAT'S ACTUALLY HAPPENING" PID DECOMPOSITION HUD
        decomp_frame = tk.Frame(self, bg="#0d111a", padx=12, pady=5, highlightthickness=1, highlightbackground="#1e293b")
        decomp_frame.pack(fill=tk.X, padx=10, pady=2)

        # Left: Live P, I, D Action Bars
        bars_box = tk.Frame(decomp_frame, bg="#0d111a")
        bars_box.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.bar_p_canvas = self.make_action_meter(bars_box, "P-Action (Spring Restoring Force):", "#00f2fe", 0)
        self.bar_i_canvas = self.make_action_meter(bars_box, "I-Action (Steady-State Integrator):", "#38bdf8", 1)
        self.bar_d_canvas = self.make_action_meter(bars_box, "D-Action (Velocity Damper/Braking):", "#fbbf24", 2)

        # Right: Real-time Theory Insight Ticker
        insight_box = tk.Frame(decomp_frame, bg="#131926", padx=10, pady=4, highlightthickness=1, highlightbackground="#1f293d")
        insight_box.pack(side=tk.RIGHT, padx=5)

        tk.Label(insight_box, text="REAL-TIME CONTROL INSIGHT:", fg="#64748b", bg="#131926", font=("Segoe UI", 7, "bold")).pack(anchor="w")
        self.insight_lbl = tk.Label(insight_box, text=self.insight_text, fg="#38bdf8", bg="#131926", font=("Segoe UI", 8, "bold"), wraplength=380, justify="left")
        self.insight_lbl.pack(anchor="w")

        # 4. CENTER DISPLAY: ANIMATION CANVAS + OSCILLOSCOPE
        display_frame = tk.Frame(self, bg="#080a0f", padx=10, pady=2)
        display_frame.pack(fill=tk.BOTH, expand=True)

        # Animation Viewport
        self.anim_frame = tk.Frame(display_frame, bg="#0d111a", highlightthickness=1, highlightbackground="#1e293b")
        self.anim_frame.pack(fill=tk.X, pady=3)

        anim_hud = tk.Frame(self.anim_frame, bg="#0d111a", padx=8, pady=2)
        anim_hud.pack(fill=tk.X)
        self.anim_title_lbl = tk.Label(anim_hud, text="ENVIRONMENT: 🌿 AUTOMATED GREENHOUSE - ULTRASONIC PLANT CANOPY DISTANCE CONTROL", fg="#10b981", bg="#0d111a", font=("Segoe UI", 8, "bold"))
        self.anim_title_lbl.pack(side=tk.LEFT)
        self.anim_status_lbl = tk.Label(anim_hud, text="APPLICATION: AUTOMATED SPRAY NOZZLE / LIGHTING BOOM HEIGHT REGULATION", fg="#64748b", bg="#0d111a", font=("Segoe UI", 7, "bold"))
        self.anim_status_lbl.pack(side=tk.RIGHT)

        self.anim_canvas = tk.Canvas(self.anim_frame, height=175, bg="#05070c", highlightthickness=0)
        self.anim_canvas.pack(fill=tk.X, expand=True)

        # Oscilloscope Viewport
        scope_frame = tk.Frame(display_frame, bg="#0d111a", highlightthickness=1, highlightbackground="#1e293b")
        scope_frame.pack(fill=tk.BOTH, expand=True, pady=3)

        scope_hud = tk.Frame(scope_frame, bg="#0d111a", padx=8, pady=3)
        scope_hud.pack(fill=tk.X)

        tk.Label(scope_hud, text="DYNAMIC STEP RESPONSE (DIGITAL PHOSPHOR OSCILLOSCOPE)", fg="#e2e8f0", bg="#0d111a", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)
        tk.Label(scope_hud, text="■ Setpoint r(t)", fg="#00f2fe", bg="#0d111a", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=10)
        tk.Label(scope_hud, text="■ Output y(t)", fg="#38bdf8", bg="#0d111a", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=6)
        tk.Label(scope_hud, text="■ Effort u(t)", fg="#fbbf24", bg="#0d111a", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=6)

        self.scope_metrics_lbl = tk.Label(scope_hud, text="tr: 0.85s | ts: 1.82s | Mp: 7.8% | ess: 0.00cm", fg="#10b981", bg="#0d111a", font=("Segoe UI", 8, "bold"))
        self.scope_metrics_lbl.pack(side=tk.RIGHT, padx=6)

        self.scope_canvas = tk.Canvas(scope_frame, bg="#040609", highlightthickness=0)
        self.scope_canvas.pack(fill=tk.BOTH, expand=True)

        # 5. BOTTOM COCKPIT BENCH CONTROLS
        ctrl_frame = tk.Frame(self, bg="#0d111a", padx=12, pady=6, highlightthickness=1, highlightbackground="#1e293b")
        ctrl_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=6)

        # Tuning Sliders Row
        sliders_box = tk.Frame(ctrl_frame, bg="#0d111a")
        sliders_box.pack(fill=tk.X)

        self.slider_kp = self.add_styled_slider(sliders_box, "PROPORTIONAL (Kp):", 0.0, 10.0, self.kp, 0.1, "#00f2fe", "P", 0)
        self.slider_ki = self.add_styled_slider(sliders_box, "INTEGRAL (Ki):", 0.0, 0.5, self.ki, 0.01, "#38bdf8", "I", 1)
        self.slider_kd = self.add_styled_slider(sliders_box, "DERIVATIVE (Kd):", 0.0, 3.0, self.kd, 0.05, "#fbbf24", "D", 2)
        self.slider_target = self.add_styled_slider(sliders_box, "TARGET SETPOINT (cm):", 5.0, 45.0, self.target, 1.0, "#a78bfa", "T", 3)
        self.slider_noise = self.add_styled_slider(sliders_box, "SENSOR NOISE JITTER:", 0.0, 2.0, self.noise_amplitude, 0.1, "#f43f5e", "N", 4)

        # Step Buttons & Disturbance Impulse Row
        actions_row = tk.Frame(ctrl_frame, bg="#0d111a", pady=3)
        actions_row.pack(fill=tk.X)

        tk.Label(actions_row, text="DYNAMIC TEST PROBES:", fg="#64748b", bg="#0d111a", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT, padx=3)
        for target_val in [15.0, 25.0, 35.0]:
            btn = tk.Button(actions_row, text=f"Step {int(target_val)} cm", command=lambda v=target_val: self.trigger_step(v), bg="#1e293b", fg="#00f2fe", relief="flat", padx=8, pady=2, font=("Segoe UI", 8, "bold"), cursor="hand2", activebackground="#334155")
            btn.pack(side=tk.LEFT, padx=2)

        tk.Label(actions_row, text="|", fg="#334155", bg="#0d111a").pack(side=tk.LEFT, padx=6)

        # Disturbance Button
        btn_nudge = tk.Button(actions_row, text="⚡ INJECT PHYSICAL DISTURBANCE (+8 cm NUDGE)", command=self.inject_nudge, bg="#be123c", fg="#ffffff", relief="flat", padx=10, pady=2, font=("Segoe UI", 8, "bold"), cursor="hand2", activebackground="#e11d48")
        btn_nudge.pack(side=tk.LEFT, padx=3)

        btn_rst_int = tk.Button(actions_row, text="Reset Integrator", command=self.reset_integrator, bg="#334155", fg="#cbd5e1", relief="flat", padx=7, pady=2, font=("Segoe UI", 7), cursor="hand2")
        btn_rst_int.pack(side=tk.LEFT, padx=3)

    def create_badge(self, parent, title, val, color, col):
        card = tk.Frame(parent, bg="#0d111a", padx=8, pady=5, highlightthickness=1, highlightbackground="#1e293b")
        card.grid(row=0, column=col, padx=3, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        accent = tk.Frame(card, bg=color, height=2)
        accent.pack(fill=tk.X, side=tk.TOP, pady=(0, 3))

        tk.Label(card, text=title, fg="#64748b", bg="#0d111a", font=("Segoe UI", 7, "bold")).pack(anchor="w")
        lbl = tk.Label(card, text=val, fg=color, bg="#0d111a", font=("Segoe UI", 11, "bold"))
        lbl.pack(anchor="w", pady=1)
        return lbl

    def make_action_meter(self, parent, label_text, color, col):
        frame = tk.Frame(parent, bg="#0d111a", padx=4)
        frame.grid(row=0, column=col, sticky="ew")
        parent.grid_columnconfigure(col, weight=1)

        hdr = tk.Frame(frame, bg="#0d111a")
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text=label_text, fg="#94a3b8", bg="#0d111a", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT)
        val_lbl = tk.Label(hdr, text="0.0 N", fg=color, bg="#0d111a", font=("Segoe UI", 7, "bold"))
        val_lbl.pack(side=tk.RIGHT)

        canvas = tk.Canvas(frame, height=10, bg="#05070c", highlightthickness=1, highlightbackground="#1e293b")
        canvas.pack(fill=tk.X, pady=2)
        canvas.val_lbl = val_lbl
        canvas.color = color
        return canvas

    def add_styled_slider(self, parent, label_text, from_val, to_val, default_val, res, color, param_tag, col):
        frame = tk.Frame(parent, bg="#0d111a", padx=3)
        frame.grid(row=0, column=col, padx=3, sticky="ew")
        parent.grid_columnconfigure(col, weight=1)

        val_var = tk.DoubleVar(value=default_val)
        hdr = tk.Frame(frame, bg="#0d111a")
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text=label_text, fg="#94a3b8", bg="#0d111a", font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT)
        val_lbl = tk.Label(hdr, text=f"{default_val:.2f}", fg=color, bg="#0d111a", font=("Segoe UI", 8, "bold"))
        val_lbl.pack(side=tk.RIGHT)

        def on_slide(val):
            v = float(val)
            val_lbl.config(text=f"{v:.2f}")
            self.on_slider_change(param_tag, v)

        scale = tk.Scale(frame, from_=from_val, to=to_val, resolution=res, variable=val_var, orient=tk.HORIZONTAL, showvalue=False, command=on_slide, bg="#1e293b", fg=color, highlightthickness=0, troughcolor="#05070c", activebackground=color, cursor="hand2")
        scale.pack(fill=tk.X, pady=1)
        scale.var = val_var
        return scale

    def on_slider_change(self, tag, val):
        if tag == "P":
            self.kp = val
            if self.kp > 4.5:
                self.insight_text = f"Kp set to {val:.1f} (HIGH): Aggressive stiffness! Speeds up rise time, but increases oscillations."
                self.insight_color = "#f43f5e"
            else:
                self.insight_text = f"Kp set to {val:.1f}: Restoring spring force adjusted proportionally to error."
                self.insight_color = "#00f2fe"
            # Induce a slight disturbance so the user immediately sees the change in behavior!
            self.velocity += 1.5 * (1.0 if random.random() > 0.5 else -1.0)

        elif tag == "I":
            self.ki = val
            if self.ki > 0.2:
                self.insight_text = f"Ki set to {val:.2f} (STRONG): High integral accumulation! Destroys steady-state error, may cause slow hunting."
                self.insight_color = "#38bdf8"
            else:
                self.insight_text = f"Ki set to {val:.2f}: Integrator summing past errors to eliminate steady-state offset."
                self.insight_color = "#38bdf8"

        elif tag == "D":
            self.kd = val
            if self.kd > 1.5:
                self.insight_text = f"Kd set to {val:.2f} (HEAVY DAMPING): Acts like high-viscosity shock absorber. Quenches overshoot!"
                self.insight_color = "#fbbf24"
            else:
                self.insight_text = f"Kd set to {val:.2f}: Derivative damper resisting rapid changes in velocity (de/dt)."
                self.insight_color = "#fbbf24"

        elif tag == "T":
            self.trigger_step(val)
            self.insight_text = f"Target Setpoint adjusted to {val:.1f} cm. System responding with transient step maneuver."
            self.insight_color = "#a78bfa"

        elif tag == "N":
            self.noise_amplitude = val
            self.insight_text = f"Acoustic Noise Jitter set to {val:.2f} cm. Exponential Moving Average filter actively smoothing echoes."
            self.insight_color = "#f43f5e"

        self.insight_lbl.config(text=self.insight_text, fg=self.insight_color)

        if self.mode == "hw" and self.is_hw_connected and self.ser:
            try:
                self.ser.write(f"P{self.kp}\nI{self.ki}\nD{self.kd}\nT{self.target}\n".encode())
            except Exception:
                pass

    def toggle_system_power(self):
        self.system_enabled = not self.system_enabled
        if self.system_enabled:
            self.btn_power.config(text="● SYSTEM ACTIVE", bg="#059669", activebackground="#10b981")
            self.card_mode.config(text=f"{self.controller_type} ACTIVE", fg="#10b981")
            self.insight_text = "System Energized: Actuator driving closed-loop feedback loop."
        else:
            self.btn_power.config(text="● SYSTEM IDLE (OFF)", bg="#be123c", activebackground="#e11d48")
            self.card_mode.config(text="PAUSED / IDLE", fg="#f43f5e")
            self.control_output = 0.0
            self.insight_text = "System De-energized: Actuator output forced to 0. Mechanical plant coasting to rest."
        self.insight_lbl.config(text=self.insight_text, fg="#f43f5e" if not self.system_enabled else "#10b981")

    def toggle_controller_algo(self):
        if self.controller_type == "PID":
            self.controller_type = "ON_OFF"
            self.btn_algo.config(text="MODE: ON/OFF BANG-BANG", fg="#f43f5e")
            self.card_mode.config(text="BANG-BANG ACTIVE", fg="#f43f5e")
            self.insight_text = "ON/OFF Bang-Bang Active: Actuator slams between full saturation (+/- 35). Produces sustained limit cycles!"
            self.insight_color = "#f43f5e"
        else:
            self.controller_type = "PID"
            self.btn_algo.config(text="MODE: PID CONTROLLER", fg="#38bdf8")
            self.card_mode.config(text="PID OPTIMAL", fg="#10b981")
            self.insight_text = "PID Controller Active: Smooth proportional moderation with integral elimination of offset and derivative damping."
            self.insight_color = "#10b981"
        self.insight_lbl.config(text=self.insight_text, fg=self.insight_color)
        self.trigger_step(self.target)

    def cycle_env_view(self):
        """Cycles between: 🌿 Botanical Plants -> 🚗 Vehicle ACC -> 🔬 Lab Bench"""
        if self.env_view == "plant":
            self.env_view = "vehicle"
            self.btn_env.config(text="ENV: 🚗 ROAD / VEHICLE ACC", bg="#2563eb", activebackground="#3b82f6")
            self.anim_title_lbl.config(text="ENVIRONMENT: 🚗 AUTONOMOUS ELECTRIC VEHICLE - ADAPTIVE CRUISE CONTROL (ACC)", fg="#38bdf8")
            self.anim_status_lbl.config(text="APPLICATION: HIGHWAY INTER-VEHICLE HEADWAY SPACING & COLLISION MITIGATION")
            self.sub_header_lbl.config(text="Active Context: 🚗 Real-World Autonomous Driving & Headway Control", fg="#38bdf8")
        elif self.env_view == "vehicle":
            self.env_view = "bench"
            self.btn_env.config(text="ENV: 🔬 LAB BENCH APPARATUS", bg="#7c3aed", activebackground="#8b5cf6")
            self.anim_title_lbl.config(text="ENVIRONMENT: 🔬 EMBEDDED LABORATORY BENCH - HC-SR04 & MICRO SERVO TRACK", fg="#a78bfa")
            self.anim_status_lbl.config(text="APPLICATION: 1-DOF PHYSICAL EXPERIMENTAL TESTBED (ARDUINO ATMEGA328P)")
            self.sub_header_lbl.config(text="Active Context: 🔬 Laboratory Embedded Hardware Prototype (Arduino)", fg="#a78bfa")
        else:
            self.env_view = "plant"
            self.btn_env.config(text="ENV: 🌿 GREENHOUSE PLANTS", bg="#047857", activebackground="#059669")
            self.anim_title_lbl.config(text="ENVIRONMENT: 🌿 AUTOMATED GREENHOUSE - ULTRASONIC PLANT CANOPY DISTANCE CONTROL", fg="#10b981")
            self.anim_status_lbl.config(text="APPLICATION: AUTOMATED SPRAY NOZZLE / LIGHTING BOOM HEIGHT REGULATION", fg="#64748b")
            self.sub_header_lbl.config(text="Active Context: 🌿 Smart Agriculture / Botanical Canopy Distance Keeping", fg="#10b981")

    def trigger_step(self, new_target):
        self.target = new_target
        self.slider_target.var.set(new_target)
        self.step_start_time = time.time()
        self.step_initial_pos = self.position
        self.step_monitoring = True
        self.rise_time = 0.0
        self.settling_time = 0.0
        self.peak_overshoot = 0.0
        if self.mode == "hw" and self.is_hw_connected and self.ser:
            try:
                self.ser.write(f"T{new_target}\n".encode())
            except Exception:
                pass

    def inject_nudge(self):
        self.position = min(48.0, self.position + 8.0)
        self.velocity += 18.0
        self.step_start_time = time.time()
        self.step_monitoring = True
        self.insight_text = "Physical Impulse Injected (+8 cm): Derivative term immediately reacting to velocity jump (de/dt) to arrest displacement!"
        self.insight_color = "#f43f5e"
        self.insight_lbl.config(text=self.insight_text, fg=self.insight_color)

    def reset_integrator(self):
        self.integral = 0.0
        self.insight_text = "Integrator Reset: Cumulative past error cleared to zero."
        self.insight_lbl.config(text=self.insight_text, fg="#cbd5e1")

    def refresh_ports(self):
        if SERIAL_AVAILABLE:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                self.port_combo.current(0)
        else:
            self.port_combo['values'] = ["No COM"]

    def toggle_hw_connection(self):
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
                self.btn_hw_connect.config(text="DISCONNECT", bg="#be123c", fg="white")
                threading.Thread(target=self.hw_read_loop, daemon=True).start()
            except Exception as e:
                messagebox.showerror("Serial Error", str(e))
        else:
            self.is_hw_connected = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.btn_hw_connect.config(text="USB", bg="#1e293b", fg="#94a3b8")
            self.mode = "sim"

    def hw_read_loop(self):
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
                    self.control_output = d.get("Output", 0.0)
                    self.p_term = d.get("P", 0.0)
                    self.i_term = d.get("I", 0.0)
                    self.d_term = d.get("D", 0.0)
            except Exception:
                pass
            time.sleep(0.01)

    def physics_step(self, dt):
        """Simulates 2nd-order dynamic physical plant with noise and selected controller"""
        if not self.system_enabled:
            self.control_output = 0.0
            self.p_term = 0.0
            self.i_term = 0.0
            self.d_term = 0.0
            self.servo_angle = 90.0
            self.velocity *= 0.94
            self.position += self.velocity * dt
            return

        # 1. Ultrasonic acoustic measurement with realistic jitter
        jitter = random.gauss(0, self.noise_amplitude) if self.noise_amplitude > 0 else 0.0
        self.raw_dist = max(2.0, min(50.0, self.position + jitter))

        # 2. Pre-filter (Exponential Moving Average)
        self.smoothed_dist = (self.alpha_ema * self.raw_dist) + ((1.0 - self.alpha_ema) * self.smoothed_dist)
        error = self.target - self.smoothed_dist

        # 3. Controller Execution: PID vs ON/OFF Bang-Bang
        if self.controller_type == "ON_OFF":
            deadband = 0.8
            if error > deadband:
                self.control_output = +35.0
            elif error < -deadband:
                self.control_output = -35.0
            else:
                self.control_output = 0.0
            self.p_term = self.control_output
            self.i_term = 0.0
            self.d_term = 0.0
            self.servo_angle = 90.0 + self.control_output
        else:
            ctrl_error = 0.0 if abs(error) < 0.6 else error
            self.p_term = self.kp * ctrl_error

            # Integral with Anti-Windup
            self.integral += ctrl_error * dt
            self.integral = max(-40.0, min(40.0, self.integral))
            self.i_term = self.ki * self.integral

            # Derivative with Low-Pass Filter
            raw_deriv = (ctrl_error - self.prev_error) / dt if dt > 0 else 0.0
            self.filtered_derivative = (self.alpha_deriv * raw_deriv) + ((1.0 - self.alpha_deriv) * self.filtered_derivative)
            self.d_term = self.kd * self.filtered_derivative
            self.prev_error = ctrl_error

            self.control_output = self.p_term + self.i_term + self.d_term
            self.servo_angle = max(20.0, min(160.0, 90.0 + self.control_output))

        # 4. Second-Order Plant Dynamics
        tilt_rad = math.radians(self.servo_angle - 90.0)
        actuator_force = 125.0 * math.sin(tilt_rad)
        damping_force = -self.plant_damping * self.velocity

        net_force = actuator_force + damping_force
        acceleration = net_force / self.plant_mass

        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        if self.position <= 2.0:
            self.position = 2.0
            self.velocity = 0.0
        elif self.position >= 50.0:
            self.position = 50.0
            self.velocity = 0.0

        # 5. Calculate transient response metrics
        if self.step_monitoring:
            elapsed = time.time() - self.step_start_time
            step_mag = abs(self.target - self.step_initial_pos)
            if step_mag > 1.0:
                if self.rise_time == 0.0:
                    fraction = (self.position - self.step_initial_pos) / (self.target - self.step_initial_pos)
                    if fraction >= 0.90:
                        self.rise_time = elapsed

                if (self.target > self.step_initial_pos and self.position > self.target) or (self.target < self.step_initial_pos and self.position < self.target):
                    overshoot_cm = abs(self.position - self.target)
                    pct = (overshoot_cm / step_mag) * 100.0
                    if pct > self.peak_overshoot:
                        self.peak_overshoot = pct

                if abs(error) <= (0.05 * step_mag) and elapsed > 0.6:
                    if self.settling_time == 0.0:
                        self.settling_time = elapsed
                else:
                    if elapsed > self.settling_time > 0.0 and abs(error) > (0.05 * step_mag):
                        self.settling_time = 0.0

        self.steady_state_error = abs(error)

    def draw_meters(self):
        """Draws live dynamic P, I, D Action Contribution Bars"""
        # P Meter
        self.render_action_bar(self.bar_p_canvas, self.p_term, 30.0, "#00f2fe")
        # I Meter
        self.render_action_bar(self.bar_i_canvas, self.i_term, 20.0, "#38bdf8")
        # D Meter
        self.render_action_bar(self.bar_d_canvas, self.d_term, 20.0, "#fbbf24")

    def render_action_bar(self, c, val, max_scale, color):
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 20 or h < 6:
            return

        mid_x = w / 2
        norm = max(-1.0, min(1.0, val / max_scale))
        bar_w = norm * (w / 2 - 4)

        # Center line
        c.create_line(mid_x, 0, mid_x, h, fill="#334155", width=1)

        # Active bar
        if bar_w > 0:
            c.create_rectangle(mid_x, 2, mid_x + bar_w, h - 2, fill=color, outline="")
        elif bar_w < 0:
            c.create_rectangle(mid_x + bar_w, 2, mid_x, h - 2, fill=color, outline="")

        sign = "+" if val > 0 else ""
        c.val_lbl.config(text=f"{sign}{val:.1f} N")

    def draw_animation(self):
        c = self.anim_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        if self.env_view == "plant":
            self.draw_greenhouse_plant_environment(c, w, h)
        elif self.env_view == "vehicle":
            self.draw_vehicle_environment(c, w, h)
        else:
            self.draw_bench_apparatus(c, w, h)

    def draw_greenhouse_plant_environment(self, c, w, h):
        """Draws realistic Botanical Greenhouse: potted plants, foliage, overhead boom with ultrasonic nozzle"""
        ground_y = h - 28

        # 1. Greenhouse Architecture (Glass panels and steel trusses)
        c.create_rectangle(0, 0, w, ground_y, fill="#051410", outline="")
        for glass_x in range(0, w, 110):
            c.create_line(glass_x, 0, glass_x + 50, ground_y, fill="#0d2b20", width=1)
            c.create_line(glass_x + 50, 0, glass_x, ground_y, fill="#0d2b20", width=1)

        # Greenhouse floor (soil / brick pavers)
        c.create_rectangle(0, ground_y, w, h, fill="#2b1810", outline="#45271a")

        # Track mapping
        track_start_x = 130
        track_end_x = w - 120
        track_len = track_end_x - track_start_x

        # 2. Automated Ultrasonic Boom Carriage (Left side: Mounted on gantry)
        gantry_x = track_start_x - 55
        gantry_y = ground_y - 45

        # Overhead aluminum gantry rail
        c.create_line(gantry_x - 40, gantry_y - 25, w - 20, gantry_y - 25, fill="#64748b", width=3)

        # Boom Carriage
        c.create_rectangle(gantry_x - 22, gantry_y - 32, gantry_x + 22, gantry_y + 10, fill="#1e293b", outline="#38bdf8", width=2)
        # HC-SR04 Transducer pod facing right towards plants
        c.create_rectangle(gantry_x + 22, gantry_y - 14, gantry_x + 30, gantry_y + 4, fill="#0984e3", outline="#ffffff")
        c.create_oval(gantry_x + 30, gantry_y - 12, gantry_x + 36, gantry_y - 4, fill="#e2e8f0")
        c.create_oval(gantry_x + 30, gantry_y - 2, gantry_x + 36, gantry_y + 6, fill="#e2e8f0")
        # Water/Mist Nozzle on Boom
        c.create_polygon(gantry_x + 18, gantry_y + 10, gantry_x + 28, gantry_y + 18, gantry_x + 8, gantry_y + 18, fill="#fbbf24")
        c.create_text(gantry_x - 2, gantry_y - 42, text="AUTOMATED GANTRY BOOM", fill="#38bdf8", font=("Segoe UI", 7, "bold"))

        # Animated Spray Mist from nozzle
        if self.system_enabled:
            self.spray_phase = (self.spray_phase + 0.2) % 1.0
            for sp_i in range(4):
                sp_x = gantry_x + 24 + sp_i * 8
                sp_y = gantry_y + 18 + sp_i * 4
                c.create_oval(sp_x, sp_y, sp_x + 3, sp_y + 3, fill="#38bdf8", outline="")

        # 3. Botanical Foliage & Plant Canopy (Target Plant)
        pos_clamped = max(0.0, min(50.0, self.position))
        plant_x = track_start_x + (pos_clamped / 50.0) * track_len
        plant_y = ground_y

        # Gentle natural plant sway animation
        self.plant_sway = math.sin(time.time() * 2.5) * 3.0

        # Terracotta Plant Pot
        pot_w = 26
        c.create_polygon(plant_x - pot_w, plant_y, plant_x + pot_w, plant_y, plant_x + pot_w - 6, plant_y + 24, plant_x - pot_w + 6, plant_y + 24, fill="#c2410c", outline="#ea580c", width=1)
        c.create_rectangle(plant_x - pot_w - 2, plant_y - 4, plant_x + pot_w + 2, plant_y, fill="#9a3412", outline="")

        # Plant Stems and Lush Foliage
        stem_top_x = plant_x + self.plant_sway
        stem_top_y = plant_y - 65
        c.create_line(plant_x, plant_y, stem_top_x, stem_top_y, fill="#15803d", width=4)

        # Big Leafy Canopy (Monstera / Ficus style leaves)
        for leaf_offset in [-25, 0, 25]:
            lx = stem_top_x + leaf_offset
            ly = stem_top_y + abs(leaf_offset) * 0.4
            c.create_oval(lx - 18, ly - 14, lx + 18, ly + 14, fill="#16a34a", outline="#4ade80", width=1.5)
            c.create_line(lx - 12, ly, lx + 12, ly, fill="#86efac", width=1)

        c.create_text(plant_x, stem_top_y - 24, text=f"PLANT CANOPY ({self.position:.1f} cm)", fill="#4ade80", font=("Segoe UI", 8, "bold"))

        # 4. Target Setpoint Flag Marker
        setpoint_clamped = max(0.0, min(50.0, self.target))
        sp_x = track_start_x + (setpoint_clamped / 50.0) * track_len
        c.create_line(sp_x, ground_y - 75, sp_x, ground_y, fill="#00f2fe", width=2, dash=(3, 3))
        c.create_polygon(sp_x, ground_y - 75, sp_x + 14, ground_y - 67, sp_x, ground_y - 59, fill="#00f2fe")
        c.create_text(sp_x, ground_y - 84, text=f"DESIRED GAP: {self.target:.1f} cm", fill="#00f2fe", font=("Segoe UI", 8, "bold"))

        # 5. Animated Sonar Waves Bouncing off Plant Canopy Leaves
        if self.system_enabled:
            self.wave_phase = (self.wave_phase + 0.16) % 1.0
            gap_span = (plant_x - 18 - (gantry_x + 36))
            if gap_span > 20:
                for arc_i in range(3):
                    prog = (self.wave_phase + arc_i * 0.33) % 1.0
                    ax = (gantry_x + 36) + (gap_span * prog)
                    ay = gantry_y - 4
                    r_span = 12 + 16 * prog
                    c.create_arc(ax - r_span, ay - r_span, ax + r_span, ay + r_span, start=-45, extent=90, style=tk.ARC, outline="#00f2fe", width=2)

        # 6. Real-time Force Vectors on Plant
        if abs(self.control_output) > 1.0:
            force_len = max(-45, min(45, self.control_output * 1.5))
            f_col = "#00f2fe" if force_len > 0 else "#f43f5e"
            c.create_line(plant_x, stem_top_y, plant_x + force_len, stem_top_y, fill=f_col, width=3, arrow=tk.LAST)
            c.create_text(plant_x + force_len / 2, stem_top_y - 12, text="F_PID", fill=f_col, font=("Segoe UI", 7, "bold"))

    def draw_vehicle_environment(self, c, w, h):
        """Draws realistic automotive scene: Highway, Autonomous Cyber-Rover with Sonar, Lead Car"""
        horizon_y = int(h * 0.40)
        road_bottom_y = h

        # Sky Gradient
        sky_colors = ["#090d16", "#0f172a", "#1e293b", "#334155", "#475569"]
        for idx, col in enumerate(sky_colors):
            y1 = int(horizon_y * (idx / len(sky_colors)))
            y2 = int(horizon_y * ((idx + 1) / len(sky_colors)))
            c.create_rectangle(0, y1, w, y2, fill=col, outline="")

        # Mountains
        c.create_polygon(0, horizon_y, 90, horizon_y - 25, 210, horizon_y, fill="#0f172a", outline="")
        c.create_polygon(160, horizon_y, 310, horizon_y - 35, 460, horizon_y, fill="#131b2e", outline="")
        c.create_polygon(400, horizon_y, 580, horizon_y - 30, 760, horizon_y, fill="#0f172a", outline="")
        c.create_polygon(700, horizon_y, 880, horizon_y - 40, w, horizon_y, fill="#131b2e", outline="")

        # Roadside Verge
        c.create_rectangle(0, horizon_y, w, road_bottom_y, fill="#064e3b", outline="")

        # Highway Asphalt
        road_top_w = int(w * 0.78)
        road_left_top = (w - road_top_w) // 2
        road_right_top = road_left_top + road_top_w
        c.create_polygon(road_left_top, horizon_y, road_right_top, horizon_y, w - 20, road_bottom_y, 20, road_bottom_y, fill="#1e2430", outline="#334155", width=2)

        # Center Divider Dashes
        self.road_scroll = (self.road_scroll + (0.9 if self.system_enabled else 0.0)) % 40.0
        center_y = int(horizon_y + (road_bottom_y - horizon_y) * 0.65)
        for x_step in range(-40, w + 60, 50):
            dash_x = x_step + self.road_scroll
            c.create_line(dash_x, center_y, dash_x + 22, center_y, fill="#fbbf24", width=3)

        track_start_x = 140
        track_end_x = w - 120
        track_len = track_end_x - track_start_x

        # Follower Autonomous AGV
        car_x = track_start_x - 55
        car_y = center_y - 8

        if self.system_enabled:
            c.create_polygon(car_x + 38, car_y - 4, car_x + 190, car_y - 25, car_x + 190, car_y + 25, car_x + 38, car_y + 6, fill="#0c2340", outline="")

        c.create_rectangle(car_x - 42, car_y - 16, car_x + 36, car_y + 14, fill="#1e3a8a", outline="#60a5fa", width=2)
        c.create_polygon(car_x - 22, car_y - 16, car_x - 8, car_y - 28, car_x + 18, car_y - 28, car_x + 28, car_y - 16, fill="#0f172a", outline="#93c5fd", width=1)
        c.create_rectangle(car_x + 34, car_y - 12, car_x + 38, car_y - 4, fill="#00f2fe", outline="#ffffff")
        c.create_rectangle(car_x + 34, car_y + 4, car_x + 38, car_y + 12, fill="#00f2fe", outline="#ffffff")

        # Wheels
        for wx in [car_x - 26, car_x + 20]:
            wy = car_y + 14
            c.create_oval(wx - 9, wy - 9, wx + 9, wy + 9, fill="#0f172a", outline="#94a3b8", width=2)
            c.create_oval(wx - 4, wy - 4, wx + 4, wy + 4, fill="#00f2fe", outline="")

        c.create_text(car_x - 4, car_y - 34, text="AUTONOMOUS AGV", fill="#60a5fa", font=("Segoe UI", 8, "bold"))

        # Lead Vehicle
        pos_clamped = max(0.0, min(50.0, self.position))
        target_x = track_start_x + (pos_clamped / 50.0) * track_len
        target_y = center_y - 8

        c.create_rectangle(target_x - 20, target_y - 14, target_x + 40, target_y + 14, fill="#881337", outline="#fb7185", width=2)
        c.create_polygon(target_x - 6, target_y - 14, target_x + 6, target_y - 25, target_x + 26, target_y - 25, target_x + 34, target_y - 14, fill="#0f172a", outline="#f43f5e", width=1)
        brake_glow = "#f43f5e" if self.velocity < 0 else "#be123c"
        c.create_rectangle(target_x - 23, target_y - 12, target_x - 20, target_y + 12, fill=brake_glow, outline="#fda4af")
        c.create_text(target_x + 12, target_y - 32, text=f"LEAD VEHICLE ({self.position:.1f} m)", fill="#fb7185", font=("Segoe UI", 8, "bold"))

        # Sonar Wave Arcs
        if self.system_enabled:
            self.wave_phase = (self.wave_phase + 0.16) % 1.0
            gap_span = (target_x - (car_x + 44))
            if gap_span > 20:
                for arc_i in range(3):
                    prog = (self.wave_phase + arc_i * 0.33) % 1.0
                    ax = (car_x + 44) + (gap_span * prog)
                    ay = car_y
                    r_span = 14 + 18 * prog
                    c.create_arc(ax - r_span, ay - r_span, ax + r_span, ay + r_span, start=-45, extent=90, style=tk.ARC, outline="#00f2fe", width=2)

    def draw_bench_apparatus(self, c, w, h):
        """Photorealistic Laboratory Test Track with Aluminum Rails and HC-SR04 Transducer"""
        track_y = h - 45
        track_start_x = 100
        track_end_x = w - 60
        track_len = track_end_x - track_start_x

        c.create_rectangle(15, 15, w - 15, h - 15, fill="#0b0e14", outline="#1e293b", width=1)
        c.create_rectangle(track_start_x - 20, track_y - 8, track_end_x + 20, track_y + 8, fill="#334155", outline="#64748b", width=2)
        c.create_line(track_start_x - 20, track_y, track_end_x + 20, track_y, fill="#94a3b8", width=1)

        for tick_cm in range(0, 55, 5):
            tick_x = track_start_x + (tick_cm / 50.0) * track_len
            is_major = (tick_cm % 10 == 0)
            t_len = 10 if is_major else 5
            t_col = "#38bdf8" if is_major else "#475569"
            c.create_line(tick_x, track_y + 8, tick_x, track_y + 8 + t_len, fill=t_col, width=1)
            if is_major:
                c.create_text(tick_x, track_y + 26, text=f"{tick_cm} cm", fill="#94a3b8", font=("Segoe UI", 7, "bold"))

        sensor_x = track_start_x - 55
        sensor_y = track_y - 32
        c.create_rectangle(sensor_x - 20, sensor_y - 28, sensor_x + 20, sensor_y + 28, fill="#1d4ed8", outline="#60a5fa", width=2)
        c.create_oval(sensor_x - 14, sensor_y - 22, sensor_x + 14, sensor_y - 4, fill="#e2e8f0", outline="#94a3b8", width=2)
        c.create_oval(sensor_x - 14, sensor_y + 2, sensor_x + 14, sensor_y + 20, fill="#e2e8f0", outline="#94a3b8", width=2)
        c.create_text(sensor_x, sensor_y - 36, text="HC-SR04 SENSOR", fill="#38bdf8", font=("Segoe UI", 8, "bold"))

        target_clamped = max(0.0, min(50.0, self.target))
        target_x = track_start_x + (target_clamped / 50.0) * track_len
        c.create_line(target_x, track_y - 65, target_x, track_y, fill="#00f2fe", width=2, dash=(3, 3))
        c.create_polygon(target_x, track_y - 65, target_x + 16, track_y - 55, target_x, track_y - 45, fill="#00f2fe", outline="#ffffff")
        c.create_text(target_x, track_y - 74, text=f"SETPOINT: {self.target:.1f} cm", fill="#00f2fe", font=("Segoe UI", 8, "bold"))

        pos_clamped = max(0.0, min(50.0, self.position))
        cart_x = track_start_x + (pos_clamped / 50.0) * track_len
        cart_y = track_y - 16

        c.create_rectangle(cart_x - 18, cart_y - 14, cart_x + 18, cart_y + 2, fill="#0284c7", outline="#38bdf8", width=2)
        c.create_oval(cart_x - 10, cart_y - 24, cart_x + 10, cart_y - 4, fill="#94a3b8", outline="#f1f5f9", width=2)
        c.create_text(cart_x, cart_y - 36, text=f"{self.position:.1f} cm", fill="#ffffff", font=("Segoe UI", 8, "bold"))

        if self.system_enabled:
            self.wave_phase = (self.wave_phase + 0.16) % 1.0
            span_dist = (cart_x - (sensor_x + 20))
            if span_dist > 25:
                for arc_i in range(3):
                    prog = (self.wave_phase + arc_i * 0.33) % 1.0
                    ax = (sensor_x + 20) + (span_dist * prog)
                    ay = sensor_y
                    r_span = 14 + 14 * prog
                    c.create_arc(ax - r_span, ay - r_span, ax + r_span, ay + r_span, start=-50, extent=100, style=tk.ARC, outline="#38bdf8", width=2)

        # Servo
        servo_x = w - 110
        servo_y = 55
        c.create_rectangle(servo_x - 28, servo_y - 20, servo_x + 28, servo_y + 20, fill="#0369a1", outline="#38bdf8", width=2)
        ang_rad = math.radians(self.servo_angle - 180)
        horn_len = 30
        horn_end_x = servo_x + horn_len * math.cos(ang_rad)
        horn_end_y = servo_y + horn_len * math.sin(ang_rad)
        c.create_line(servo_x, servo_y, horn_end_x, horn_end_y, fill="#fbbf24", width=4)
        c.create_text(servo_x, servo_y + 30, text=f"SERVO: {self.servo_angle:.1f}°", fill="#fbbf24", font=("Segoe UI", 8, "bold"))

    def draw_oscilloscope(self):
        c = self.scope_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        y_max = 50.0

        for cm_val in [10, 20, 30, 40]:
            y_px = h - int((cm_val / y_max) * h)
            c.create_line(0, y_px, w, y_px, fill="#131b2e", dash=(2, 4))
            c.create_text(26, y_px - 8, text=f"{cm_val} cm", fill="#475569", font=("Segoe UI", 7, "bold"))

        dx = w / (self.buffer_len - 1)

        # Fill Area between Setpoint and Position
        error_poly = []
        for i in range(self.buffer_len):
            x = i * dx
            y_pos = h - (self.pos_buffer[i] / y_max) * h
            error_poly.append((x, max(0, min(h, y_pos))))
        for i in range(self.buffer_len - 1, -1, -1):
            x = i * dx
            y_tgt = h - (self.target_buffer[i] / y_max) * h
            error_poly.append((x, max(0, min(h, y_tgt))))

        if len(error_poly) >= 6:
            flat_poly = [coord for pt in error_poly for coord in pt]
            fill_col = "#1f1422" if self.controller_type == "ON_OFF" else "#0c1d2e"
            c.create_polygon(flat_poly, fill=fill_col, outline="")

        # Target Curve (Cyan)
        target_pts = []
        for i, val in enumerate(self.target_buffer):
            x = i * dx
            y = h - (val / y_max) * h
            target_pts.extend([x, max(0, min(h, y))])
        if len(target_pts) >= 4:
            c.create_line(*target_pts, fill="#00f2fe", width=2)

        # Control Effort Curve (Amber)
        ctrl_pts = []
        for i, val in enumerate(self.ctrl_buffer):
            x = i * dx
            norm_val = 25.0 + (val * 0.38)
            y = h - (norm_val / y_max) * h
            ctrl_pts.extend([x, max(0, min(h, y))])
        if len(ctrl_pts) >= 4:
            c.create_line(*ctrl_pts, fill="#fbbf24", width=1, dash=(3, 2))

        # Position Curve (Blue or Ruby)
        pos_color = "#38bdf8" if self.controller_type == "PID" else "#f43f5e"
        glow_color = "#075985" if self.controller_type == "PID" else "#881337"
        pos_pts = []
        for i, val in enumerate(self.pos_buffer):
            x = i * dx
            y = h - (val / y_max) * h
            pos_pts.extend([x, max(0, min(h, y))])
        if len(pos_pts) >= 4:
            c.create_line(*pos_pts, fill=glow_color, width=4)
            c.create_line(*pos_pts, fill=pos_color, width=2)

    def animate_loop(self):
        curr_time = time.time()
        dt = curr_time - self.last_sim_time
        self.last_sim_time = curr_time

        if dt > 0.1:
            dt = 0.03

        if self.mode == "sim":
            self.physics_step(dt)

        self.target_buffer.pop(0)
        self.target_buffer.append(self.target)

        self.pos_buffer.pop(0)
        self.pos_buffer.append(self.position)

        self.ctrl_buffer.pop(0)
        self.ctrl_buffer.append(self.control_output)

        # Update telemetry badges
        self.card_target.config(text=f"{self.target:.1f} cm")
        self.card_pos.config(text=f"{self.position:.1f} cm")
        err = self.target - self.position
        sign = "+" if err > 0 else ""
        self.card_error.config(text=f"{sign}{err:.1f} cm")

        if self.controller_type == "ON_OFF":
            self.card_settling.config(text="LIMIT CYCLE", fg="#f43f5e")
            self.card_overshoot.config(text="SUSTAINED", fg="#f43f5e")
        else:
            ts_text = f"{self.settling_time:.2f} s" if self.settling_time > 0 else "SETTLING..."
            self.card_settling.config(text=ts_text, fg="#fbbf24")
            self.card_overshoot.config(text=f"{self.peak_overshoot:.1f} %", fg="#a78bfa")

        tr_str = f"{self.rise_time:.2f}s" if self.rise_time > 0 else "--"
        ts_str = f"{self.settling_time:.2f}s" if self.settling_time > 0 else "--"
        mp_str = f"{self.peak_overshoot:.1f}%"
        ess_str = f"{self.steady_state_error:.2f}cm"
        algo_label = "PID" if self.controller_type == "PID" else "Bang-Bang"
        self.scope_metrics_lbl.config(text=f"[{algo_label}]  tr: {tr_str}  |  ts: {ts_str}  |  Mp: {mp_str}  |  e_ss: {ess_str}")

        self.draw_meters()
        self.draw_animation()
        self.draw_oscilloscope()

        self.after(28, self.animate_loop)


if __name__ == "__main__":
    app = PIDPhysicsSimulator()
    app.mainloop()
