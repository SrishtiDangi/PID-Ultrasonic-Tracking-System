"""
PID Tracking System - Live Python Dashboard
Reads telemetry from Arduino at 115200 baud and displays real-time plots + live tuning controls.
Built with standard Tkinter (zero heavy dependencies).
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class PIDDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PID Ultrasonic Tracking System - Live Telemetry & Tuning")
        self.geometry("880x620")
        self.configure(bg="#1e1e24")

        self.ser = None
        self.is_connected = False
        self.read_thread = None

        # Data buffers for plotting (last 100 points)
        self.max_points = 100
        self.target_data = [20.0] * self.max_points
        self.dist_data = [20.0] * self.max_points
        self.servo_data = [90.0] * self.max_points

        # Current values
        self.curr_target = 20.0
        self.curr_distance = 0.0
        self.curr_error = 0.0
        self.curr_servo = 90
        self.curr_p = 0.0
        self.curr_i = 0.0
        self.curr_d = 0.0

        self.create_widgets()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(50, self.update_plot)

    def create_widgets(self):
        # Header / Connection Bar
        top_bar = tk.Frame(self, bg="#2b2b36", padx=10, pady=8)
        top_bar.pack(fill=tk.X, side=tk.TOP)

        tk.Label(top_bar, text="Arduino Port:", fg="#ffffff", bg="#2b2b36", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=5)
        self.port_combobox = ttk.Combobox(top_bar, width=15, state="readonly")
        self.port_combobox.pack(side=tk.LEFT, padx=5)

        self.refresh_btn = tk.Button(top_bar, text="Refresh", command=self.refresh_ports, bg="#3b3b4f", fg="white", relief="flat", padx=8)
        self.refresh_btn.pack(side=tk.LEFT, padx=5)

        self.connect_btn = tk.Button(top_bar, text="Connect", command=self.toggle_connection, bg="#107c41", fg="white", font=("Segoe UI", 10, "bold"), relief="flat", padx=15)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        self.status_lbl = tk.Label(top_bar, text="Disconnected", fg="#ff5555", bg="#2b2b36", font=("Segoe UI", 10, "italic"))
        self.status_lbl.pack(side=tk.RIGHT, padx=10)

        # Metric Cards Frame
        cards_frame = tk.Frame(self, bg="#1e1e24", pady=10)
        cards_frame.pack(fill=tk.X)

        self.lbl_target = self.make_metric_card(cards_frame, "Target Dist", "20.0 cm", "#00d2d3", 0)
        self.lbl_distance = self.make_metric_card(cards_frame, "Measured Dist", "-- cm", "#54a0ff", 1)
        self.lbl_error = self.make_metric_card(cards_frame, "Error (e)", "-- cm", "#ff6b6b", 2)
        self.lbl_servo = self.make_metric_card(cards_frame, "Servo Angle", "90°", "#feca57", 3)

        # Canvas Real-time Graph
        graph_frame = tk.Frame(self, bg="#1e1e24", padx=15, pady=5)
        graph_frame.pack(fill=tk.BOTH, expand=True)

        legend_frame = tk.Frame(graph_frame, bg="#1e1e24")
        legend_frame.pack(fill=tk.X, pady=2)
        tk.Label(legend_frame, text="■ Target Distance", fg="#00d2d3", bg="#1e1e24", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=10)
        tk.Label(legend_frame, text="■ Actual Distance (EMA)", fg="#54a0ff", bg="#1e1e24", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=10)

        self.canvas = tk.Canvas(graph_frame, bg="#121217", highlightthickness=1, highlightbackground="#3d3d4f")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Tuning Controls at Bottom
        tune_frame = tk.LabelFrame(self, text=" Live PID Tuning (Send commands to Arduino) ", fg="#00d2d3", bg="#2b2b36", font=("Segoe UI", 10, "bold"), padx=15, pady=8)
        tune_frame.pack(fill=tk.X, padx=15, pady=10, side=tk.BOTTOM)

        # Kp, Ki, Kd, Target controls
        self.entry_p = self.make_tuning_input(tune_frame, "Kp:", "2.0", "P", 0)
        self.entry_i = self.make_tuning_input(tune_frame, "Ki:", "0.02", "I", 1)
        self.entry_d = self.make_tuning_input(tune_frame, "Kd:", "0.1", "D", 2)
        self.entry_t = self.make_tuning_input(tune_frame, "Target (cm):", "20.0", "T", 3)

    def make_metric_card(self, parent, title, initial_val, color, col):
        card = tk.Frame(parent, bg="#2b2b36", padx=12, pady=8, highlightthickness=1, highlightbackground="#3d3d4f")
        card.grid(row=0, column=col, padx=8, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        tk.Label(card, text=title, fg="#aaaaaa", bg="#2b2b36", font=("Segoe UI", 9)).pack()
        val_lbl = tk.Label(card, text=initial_val, fg=color, bg="#2b2b36", font=("Segoe UI", 16, "bold"))
        val_lbl.pack(pady=2)
        return val_lbl

    def make_tuning_input(self, parent, label_text, default_val, cmd_prefix, col):
        frame = tk.Frame(parent, bg="#2b2b36")
        frame.grid(row=0, column=col, padx=10, pady=5)

        tk.Label(frame, text=label_text, fg="#ffffff", bg="#2b2b36", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=3)
        entry = tk.Entry(frame, width=7, font=("Segoe UI", 10), justify="center")
        entry.insert(0, default_val)
        entry.pack(side=tk.LEFT, padx=4)

        btn = tk.Button(frame, text="Set", command=lambda: self.send_param(cmd_prefix, entry.get()), bg="#3b3b4f", fg="white", relief="flat", padx=6)
        btn.pack(side=tk.LEFT, padx=2)
        return entry

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combobox.get()
            if not port:
                messagebox.showerror("Error", "Please select a serial port.")
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.is_connected = True
                self.connect_btn.config(text="Disconnect", bg="#d63031")
                self.status_lbl.config(text=f"Connected to {port} @ 115200 baud", fg="#2ed573")
                self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.read_thread.start()
            except Exception as e:
                messagebox.showerror("Connection Failed", str(e))
        else:
            self.is_connected = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.connect_btn.config(text="Connect", bg="#107c41")
            self.status_lbl.config(text="Disconnected", fg="#ff5555")

    def send_param(self, prefix, val_str):
        if not self.is_connected or not self.ser:
            messagebox.showwarning("Warning", "Connect to Arduino first!")
            return
        try:
            val = float(val_str)
            cmd = f"{prefix}{val}\n"
            self.ser.write(cmd.encode())
        except ValueError:
            messagebox.showerror("Error", "Enter a valid number!")

    def read_serial_data(self):
        while self.is_connected and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # Line format: Target:20.00	Distance:15.20	Error:4.80	P:9.60	I:0.10	D:0.00	Output:9.70	Servo:99
                if "Target:" in line and "Distance:" in line:
                    parts = line.split('\t')
                    data = {}
                    for p in parts:
                        if ':' in p:
                            k, v = p.split(':', 1)
                            data[k.strip()] = float(v.strip())

                    self.curr_target = data.get("Target", self.curr_target)
                    self.curr_distance = data.get("Distance", self.curr_distance)
                    self.curr_error = data.get("Error", self.curr_error)
                    self.curr_servo = int(data.get("Servo", self.curr_servo))
                    self.curr_p = data.get("P", 0.0)
                    self.curr_i = data.get("I", 0.0)
                    self.curr_d = data.get("D", 0.0)

                    self.target_data.pop(0)
                    self.target_data.append(self.curr_target)

                    self.dist_data.pop(0)
                    self.dist_data.append(self.curr_distance)

                    self.servo_data.pop(0)
                    self.servo_data.append(self.curr_servo)
            except Exception:
                pass
            time.sleep(0.01)

    def update_plot(self):
        # Update metric cards
        if self.is_connected:
            self.lbl_target.config(text=f"{self.curr_target:.1f} cm")
            self.lbl_distance.config(text=f"{self.curr_distance:.1f} cm")
            err_sign = "+" if self.curr_error > 0 else ""
            self.lbl_error.config(text=f"{err_sign}{self.curr_error:.1f} cm")
            self.lbl_servo.config(text=f"{self.curr_servo}°")

        # Draw smooth graph on canvas
        self.canvas.delete("all")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()

        if w > 50 and h > 50:
            # Draw grid lines (0cm, 10cm, 20cm, 30cm, 40cm, 50cm)
            max_y_range = 50.0 # cm scale
            for y_val in [10, 20, 30, 40]:
                y_px = h - int((y_val / max_y_range) * h)
                self.canvas.create_line(0, y_px, w, y_px, fill="#252530", dash=(2, 4))
                self.canvas.create_text(25, y_px - 8, text=f"{y_val}cm", fill="#555566", font=("Segoe UI", 8))

            step_x = w / (self.max_points - 1)

            # Draw Target Line (Cyan)
            target_pts = []
            for i, val in enumerate(self.target_data):
                x = i * step_x
                y = h - (val / max_y_range) * h
                target_pts.extend([x, max(0, min(h, y))])
            if len(target_pts) >= 4:
                self.canvas.create_line(*target_pts, fill="#00d2d3", width=2)

            # Draw Measured Distance Line (Blue)
            dist_pts = []
            for i, val in enumerate(self.dist_data):
                x = i * step_x
                y = h - (val / max_y_range) * h
                dist_pts.extend([x, max(0, min(h, y))])
            if len(dist_pts) >= 4:
                self.canvas.create_line(*dist_pts, fill="#54a0ff", width=2)

        self.after(50, self.update_plot)

    def on_close(self):
        self.is_connected = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.destroy()

if __name__ == "__main__":
    app = PIDDashboard()
    app.mainloop()

