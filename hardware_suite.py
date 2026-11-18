"""
================================================================================
PID ULTRASONIC TRACKING SYSTEM - HARDWARE DIAGNOSTICS & TELEMETRY SUITE
"Hardware Wala Tool" for Testing Physical Arduino, Sensor, Servo & LCD
================================================================================
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class HardwareDiagnosticsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Arduino Hardware Diagnostics & Live PID Bench")
        self.geometry("980x700")
        self.configure(bg="#0b1120")
        self.minsize(880, 600)

        self.ser = None
        self.is_connected = False
        self.read_thread = None

        # Live Hardware Telemetry
        self.hw_target = 20.0
        self.hw_distance = 0.0
        self.hw_error = 0.0
        self.hw_servo = 90
        self.hw_p = 0.0
        self.hw_i = 0.0
        self.hw_d = 0.0
        self.hw_output = 0.0

        # Oscilloscope Data
        self.max_pts = 100
        self.target_hist = [20.0] * self.max_pts
        self.dist_hist = [20.0] * self.max_pts
        self.servo_hist = [90.0] * self.max_pts

        # CSV Telemetry Logger (C1)
        self.is_logging = False
        self.log_file = None
        self.log_count = 0
        self.log_start_time = 0.0
        self.log_filename = ""

        self.build_ui()
        self.refresh_ports()
        self.after(50, self.update_gui)

    def build_ui(self):
        # 1. TOP HEADER & CONNECTION BAR
        top_bar = tk.Frame(self, bg="#1e293b", padx=14, pady=10, highlightthickness=1, highlightbackground="#334155")
        top_bar.pack(fill=tk.X, side=tk.TOP)

        title_frame = tk.Frame(top_bar, bg="#1e293b")
        title_frame.pack(side=tk.LEFT)
        tk.Label(title_frame, text="HARDWARE COMPONENT TESTBED & PID BENCH", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 12, "bold")).pack(anchor="w")
        tk.Label(title_frame, text="Physical Arduino Uno / Nano Interfacing @ 115200 Baud", fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 8)).pack(anchor="w")

        conn_box = tk.Frame(top_bar, bg="#1e293b")
        conn_box.pack(side=tk.RIGHT)

        tk.Label(conn_box, text="Port:", fg="#cbd5e1", bg="#1e293b", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=4)
        self.port_combo = ttk.Combobox(conn_box, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)

        self.btn_refresh = tk.Button(conn_box, text="Refresh", command=self.refresh_ports, bg="#334155", fg="white", relief="flat", padx=8, font=("Segoe UI", 8))
        self.btn_refresh.pack(side=tk.LEFT, padx=3)

        self.btn_connect = tk.Button(conn_box, text="Connect Hardware", command=self.toggle_serial, bg="#10b981", fg="white", font=("Segoe UI", 9, "bold"), relief="flat", padx=12, pady=2, cursor="hand2")
        self.btn_connect.pack(side=tk.LEFT, padx=6)

        self.status_dot = tk.Label(conn_box, text="● Disconnected", fg="#f43f5e", bg="#1e293b", font=("Segoe UI", 9, "bold"))
        self.status_dot.pack(side=tk.LEFT, padx=6)

        # 2. PHYSICAL HARDWARE PINOUT & STATUS CARDS
        info_frame = tk.Frame(self, bg="#0b1120", padx=14, pady=8)
        info_frame.pack(fill=tk.X)

        self.card_dist = self.make_card(info_frame, "HC-SR04 SENSOR DISTANCE", "-- cm", "#10b981", 0)
        self.card_tgt = self.make_card(info_frame, "PID TARGET SETPOINT", "20.0 cm", "#38bdf8", 1)
        self.card_err = self.make_card(info_frame, "INSTANTANEOUS ERROR", "-- cm", "#f43f5e", 2)
        self.card_srv = self.make_card(info_frame, "SERVO ANGLE (PIN D6)", "90°", "#fbbf24", 3)

        # 3. SPLIT WORKBENCH: HARDWARE COMPONENT TESTER (LEFT) + REAL-TIME OSCILLOSCOPE (RIGHT)
        mid_frame = tk.Frame(self, bg="#0b1120", padx=14)
        mid_frame.pack(fill=tk.BOTH, expand=True, pady=4)

        # 3A. Left: Hardware Component Diagnostic Suite
        diag_box = tk.LabelFrame(mid_frame, text=" Hardware Component Self-Test ", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 9, "bold"), padx=10, pady=8, width=360)
        diag_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))
        diag_box.pack_propagate(False)

        # Wiring Pinout Quick Reference
        tk.Label(diag_box, text="WIRING REFERENCE:", fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(0, 2))
        wiring_txt = (
            "• HC-SR04: Trig -> Pin 9 | Echo -> Pin 10\n"
            "• Servo: Signal -> Pin 6 | 5V & GND\n"
            "• 16x2 LCD: RS->D2, E->D3, D4..D7->D4,D5,D7,D8"
        )
        tk.Label(diag_box, text=wiring_txt, fg="#cbd5e1", bg="#0f172a", font=("Consolas", 7), justify="left", padx=6, pady=4, relief="solid", bd=1).pack(fill=tk.X, pady=2)

        # Component Diagnostic Tests
        tk.Label(diag_box, text="MANUAL ACTUATOR TEST (SERVO):", fg="#fbbf24", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(8, 2))
        srv_btn_box = tk.Frame(diag_box, bg="#1e293b")
        srv_btn_box.pack(fill=tk.X, pady=2)
        tk.Button(srv_btn_box, text="Sweep Left (45°)", command=lambda: self.send_raw_cmd("S45\n"), bg="#334155", fg="white", relief="flat", padx=6, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=2)
        tk.Button(srv_btn_box, text="Center (90°)", command=lambda: self.send_raw_cmd("S90\n"), bg="#0284c7", fg="white", relief="flat", padx=6, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=2)
        tk.Button(srv_btn_box, text="Sweep Right (135°)", command=lambda: self.send_raw_cmd("S135\n"), bg="#334155", fg="white", relief="flat", padx=6, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=2)

        tk.Label(diag_box, text="LIVE SENSOR READING:", fg="#10b981", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(8, 2))
        self.live_sensor_lbl = tk.Label(diag_box, text="Distance: Waiting for USB...", fg="#10b981", bg="#0f172a", font=("Consolas", 9, "bold"), padx=6, pady=6, relief="solid", bd=1)
        self.live_sensor_lbl.pack(fill=tk.X, pady=2)

        tk.Label(diag_box, text="PID TUNING COMMANDS (OVER SERIAL):", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(8, 2))
        self.inp_p = self.make_param_row(diag_box, "Kp Gain (P):", "2.0", "P")
        self.inp_i = self.make_param_row(diag_box, "Ki Gain (I):", "0.02", "I")
        self.inp_d = self.make_param_row(diag_box, "Kd Gain (D):", "0.10", "D")
        self.inp_t = self.make_param_row(diag_box, "Target (cm):", "20.0", "T")

        # CSV Telemetry Logger (C1)
        log_box = tk.LabelFrame(diag_box, text=" CSV Telemetry Logger (C1) ", fg="#fbbf24", bg="#1e293b", font=("Segoe UI", 8, "bold"), padx=6, pady=4)
        log_box.pack(fill=tk.X, pady=(10, 2))

        self.btn_log = tk.Button(log_box, text="🔴 Start Recording CSV", command=self.toggle_logging, bg="#b91c1c", fg="white", font=("Segoe UI", 8, "bold"), relief="flat", padx=6, pady=2, cursor="hand2")
        self.btn_log.pack(fill=tk.X, pady=2)

        self.lbl_log_status = tk.Label(log_box, text="Logger: Idle (Not recording)", fg="#94a3b8", bg="#1e293b", font=("Consolas", 8))
        self.lbl_log_status.pack(anchor="w", pady=1)

        # 3B. Right: Live Oscilloscope Waveform from Physical Hardware
        graph_box = tk.LabelFrame(mid_frame, text=" Live Hardware Oscilloscope (Target vs Actual Sensor Echo) ", fg="#10b981", bg="#1e293b", font=("Segoe UI", 9, "bold"), padx=10, pady=8)
        graph_box.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        legend_bar = tk.Frame(graph_box, bg="#1e293b")
        legend_bar.pack(fill=tk.X, pady=2)
        tk.Label(legend_bar, text="■ Target Setpoint (Cyan)", fg="#38bdf8", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=8)
        tk.Label(legend_bar, text="■ Physical Sensor Echo (Green)", fg="#10b981", bg="#1e293b", font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=8)
        self.status_msg = tk.Label(legend_bar, text="Hardware Ready", fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 8))
        self.status_msg.pack(side=tk.RIGHT, padx=8)

        self.scope_canvas = tk.Canvas(graph_box, bg="#070a12", highlightthickness=1, highlightbackground="#334155")
        self.scope_canvas.pack(fill=tk.BOTH, expand=True)

    def make_card(self, parent, title, val, color, col):
        card = tk.Frame(parent, bg="#1e293b", padx=10, pady=6, highlightthickness=1, highlightbackground="#334155")
        card.grid(row=0, column=col, padx=4, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        accent = tk.Frame(card, bg=color, height=2)
        accent.pack(fill=tk.X, side=tk.TOP, pady=(0, 4))

        tk.Label(card, text=title, fg="#94a3b8", bg="#1e293b", font=("Segoe UI", 7, "bold")).pack(anchor="w")
        lbl = tk.Label(card, text=val, fg=color, bg="#1e293b", font=("Consolas", 12, "bold"))
        lbl.pack(anchor="w", pady=1)
        return lbl

    def make_param_row(self, parent, label_text, default_val, prefix):
        row = tk.Frame(parent, bg="#1e293b")
        row.pack(fill=tk.X, pady=2)
        tk.Label(row, text=label_text, fg="#cbd5e1", bg="#1e293b", font=("Segoe UI", 8)).pack(side=tk.LEFT)
        entry = tk.Entry(row, width=6, font=("Consolas", 9), justify="center")
        entry.insert(0, default_val)
        entry.pack(side=tk.RIGHT, padx=(2, 0))
        btn = tk.Button(row, text="Send", command=lambda: self.send_param(prefix, entry.get()), bg="#334155", fg="white", relief="flat", padx=6, font=("Segoe UI", 7, "bold"))
        btn.pack(side=tk.RIGHT, padx=2)
        return entry

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
        if not self.is_connected:
            port = self.port_combo.get()
            if not port or port == "No COM":
                messagebox.showerror("Port Error", "No Arduino COM Port detected. Please plug in your Arduino USB cable and click Refresh.")
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.is_connected = True
                self.btn_connect.config(text="Disconnect Hardware", bg="#e11d48")
                self.status_dot.config(text=f"● Connected ({port})", fg="#10b981")
                self.status_msg.config(text=f"Connected to Arduino on {port} @ 115200 Baud", fg="#10b981")
                self.read_thread = threading.Thread(target=self.serial_loop, daemon=True)
                self.read_thread.start()
            except Exception as e:
                messagebox.showerror("Connection Failed", str(e))
        else:
            self.is_connected = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.btn_connect.config(text="Connect Hardware", bg="#10b981")
            self.status_dot.config(text="● Disconnected", fg="#f43f5e")
            self.status_msg.config(text="Hardware Disconnected", fg="#94a3b8")

    def send_raw_cmd(self, cmd):
        if not self.is_connected or not self.ser:
            messagebox.showwarning("Warning", "Connect to Arduino first!")
            return
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            messagebox.showerror("Write Error", str(e))

    def send_param(self, prefix, val_str):
        if not self.is_connected or not self.ser:
            messagebox.showwarning("Warning", "Connect to Arduino first!")
            return
        try:
            val = float(val_str)
            cmd = f"{prefix}{val}\n"
            self.ser.write(cmd.encode())
            self.status_msg.config(text=f"Sent command: {cmd.strip()}", fg="#38bdf8")
        except ValueError:
            messagebox.showerror("Invalid Value", "Please enter a valid number.")

    def toggle_logging(self):
        if not self.is_logging:
            import os
            os.makedirs("logs", exist_ok=True)
            self.log_filename = f"logs/hardware_run_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            try:
                self.log_file = open(self.log_filename, "w", newline="")
                # Strict C1 CSV format: timestamp,setpoint,distance,error,servo_angle,Kp,Ki,Kd
                self.log_file.write("timestamp,setpoint,distance,error,servo_angle,Kp,Ki,Kd\n")
                self.log_file.flush()
                self.is_logging = True
                self.log_count = 0
                self.log_start_time = time.time()
                self.btn_log.config(text="⏹ Stop Recording CSV", bg="#0284c7")
                self.status_msg.config(text=f"Recording telemetry to {self.log_filename}", fg="#fbbf24")
            except Exception as e:
                messagebox.showerror("Logger Error", str(e))
        else:
            self.is_logging = False
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            self.btn_log.config(text="🔴 Start Recording CSV", bg="#b91c1c")
            self.status_msg.config(text=f"Saved {self.log_count} samples to {self.log_filename}", fg="#10b981")
            messagebox.showinfo("Log Saved", f"Successfully logged {self.log_count} samples to:\n{self.log_filename}")

    def serial_loop(self):
        while self.is_connected and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # Arduino telemetry format:
                # Target:20.00  Distance:15.20  Error:4.80  P:9.60  I:0.10  D:0.00  Output:9.70  Servo:99
                if "Target:" in line and "Distance:" in line:
                    parts = line.split('\t')
                    d = {}
                    for p in parts:
                        if ':' in p:
                            k, v = p.split(':', 1)
                            d[k.strip()] = float(v.strip())

                    self.hw_target = d.get("Target", self.hw_target)
                    self.hw_distance = d.get("Distance", self.hw_distance)
                    self.hw_error = d.get("Error", self.hw_error)
                    self.hw_servo = int(d.get("Servo", self.hw_servo))
                    self.hw_p = d.get("P", 0.0)
                    self.hw_i = d.get("I", 0.0)
                    self.hw_d = d.get("D", 0.0)
                    self.hw_output = d.get("Output", 0.0)

                    self.target_hist.pop(0)
                    self.target_hist.append(self.hw_target)

                    self.dist_hist.pop(0)
                    self.dist_hist.append(self.hw_distance)

                    self.servo_hist.pop(0)
                    self.servo_hist.append(self.hw_servo)

                    # Log to CSV if enabled (C1)
                    if self.is_logging and self.log_file:
                        t_rel = time.time() - self.log_start_time
                        try:
                            kp_v = float(self.inp_p.get())
                            ki_v = float(self.inp_i.get())
                            kd_v = float(self.inp_d.get())
                        except Exception:
                            kp_v, ki_v, kd_v = 2.0, 0.02, 0.10
                        row = f"{t_rel:.4f},{self.hw_target:.2f},{self.hw_distance:.2f},{self.hw_error:.2f},{self.hw_servo},{kp_v:.2f},{ki_v:.4f},{kd_v:.2f}\n"
                        self.log_file.write(row)
                        self.log_file.flush()
                        self.log_count += 1
            except Exception:
                pass
            time.sleep(0.01)

    def update_gui(self):
        if self.is_connected:
            self.card_dist.config(text=f"{self.hw_distance:.1f} cm")
            self.card_tgt.config(text=f"{self.hw_target:.1f} cm")
            sign = "+" if self.hw_error > 0 else ""
            self.card_err.config(text=f"{sign}{self.hw_error:.1f} cm")
            self.card_srv.config(text=f"{self.hw_servo}°")
            self.live_sensor_lbl.config(text=f"Live Distance: {self.hw_distance:.1f} cm  (Err: {sign}{self.hw_error:.1f})")

        if self.is_logging:
            dur = time.time() - self.log_start_time
            self.lbl_log_status.config(text=f"Recording: {self.log_count} pts ({dur:.1f}s)", fg="#fbbf24")
        else:
            self.lbl_log_status.config(text="Logger: Idle (Not recording)", fg="#94a3b8")

        # Draw real-time oscilloscope
        self.draw_scope()
        self.after(50, self.update_gui)

    def draw_scope(self):
        c = self.scope_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        y_max = 50.0 # cm scale

        for cm in [10, 20, 30, 40]:
            y_px = h - int((cm / y_max) * h)
            c.create_line(0, y_px, w, y_px, fill="#1e293b", dash=(2, 4))
            c.create_text(24, y_px - 8, text=f"{cm} cm", fill="#64748b", font=("Consolas", 7))

        dx = w / (self.max_pts - 1)

        # Target (Cyan)
        target_pts = []
        for i, val in enumerate(self.target_hist):
            x = i * dx
            y = h - (val / y_max) * h
            target_pts.extend([x, max(0, min(h, y))])
        if len(target_pts) >= 4:
            c.create_line(*target_pts, fill="#38bdf8", width=2)

        # Measured (Green)
        dist_pts = []
        for i, val in enumerate(self.dist_hist):
            x = i * dx
            y = h - (val / y_max) * h
            dist_pts.extend([x, max(0, min(h, y))])
        if len(dist_pts) >= 4:
            c.create_line(*dist_pts, fill="#10b981", width=2)


if __name__ == "__main__":
    app = HardwareDiagnosticsApp()
    app.mainloop()

