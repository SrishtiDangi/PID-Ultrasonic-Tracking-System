"""
================================================================================
PID ULTRASONIC TRACKING SYSTEM - SYSTEM IDENTIFICATION ENGINE (PART C3)
Fits plant viscous friction, servo lag, and linkage ratio to real hardware CSV
================================================================================
"""

import os
import sys
import json
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt


def load_hardware_csv(csv_path):
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Hardware log not found: {csv_path}")

    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    t = data['timestamp']
    r = data['setpoint']
    y_hw = data['distance']
    srv_hw = data['servo_angle']
    kp = float(data['Kp'][0]) if 'Kp' in data.dtype.names else 2.40
    ki = float(data['Ki'][0]) if 'Ki' in data.dtype.names else 0.04
    kd = float(data['Kd'][0]) if 'Kd' in data.dtype.names else 0.90

    return t, r, y_hw, srv_hw, kp, ki, kd


def simulate_plant(t, r, kp, ki, kd, params, y0, v0=0.0):
    """
    Simulates the closed-loop ball and beam system under candidate parameters.
    params = [damping, servo_alpha, linkage_ratio, coulomb_friction]
    """
    damping, servo_alpha, linkage_ratio, coulomb = params

    dt_nominal = np.mean(np.diff(t))
    N = len(t)
    y_sim = np.zeros(N)
    srv_sim = np.zeros(N)

    y = float(y0)
    v = float(v0)
    servo = 90.0
    integral = 0.0
    prev_err = 0.0
    filt_d = 0.0

    y_sim[0] = y
    srv_sim[0] = servo

    for k in range(1, N):
        dt = t[k] - t[k - 1]
        if dt <= 0:
            dt = dt_nominal

        # Controller calculation
        target = r[k]
        err = target - y
        integral += err * dt
        integral = np.clip(integral, -15.0, 15.0)

        raw_d = (err - prev_err) / dt if dt > 0 else 0.0
        filt_d = 0.20 * raw_d + 0.80 * filt_d
        prev_err = err

        u = kp * err + ki * integral + kd * filt_d
        target_servo = np.clip(90.0 + u, 25.0, 155.0)

        # Servo low-pass dynamic lag
        servo += servo_alpha * (target_servo - servo)
        srv_sim[k] = servo

        # Linkage kinematics: beam tilt in degrees
        beam_deg = linkage_ratio * (servo - 90.0)

        # Non-linear Ball-on-Beam acceleration: 5/7 * g * sin(alpha)
        # alpha in radians: beam_deg * pi / 180
        # 5/7 * 981 * pi / 180 ~ 12.23 cm/s^2 per degree
        coulomb_force = coulomb * np.sign(v) if abs(v) > 0.1 else 0.0
        acc = (12.23 * beam_deg) - (damping * v) - coulomb_force

        v += acc * dt
        y += v * dt
        y = np.clip(y, 2.0, 48.0)
        y_sim[k] = y

    return y_sim, srv_sim


def identify_system(csv_path="logs/sample_hardware_step_test.csv", output_plot="Docs/system_identification_fit.png"):
    print("=" * 75)
    print(f"[*] LOADING HARDWARE CSV LOG: {csv_path}")
    t, r, y_hw, srv_hw, kp, ki, kd = load_hardware_csv(csv_path)
    print(f"    Loaded {len(t)} samples spanning {t[-1] - t[0]:.2f} seconds.")
    print(f"    Recorded Gains: Kp = {kp:.2f}, Ki = {ki:.4f}, Kd = {kd:.2f}")

    y0 = y_hw[0]

    # Baseline uncalibrated parameters [damping, servo_alpha, linkage_ratio, coulomb]
    base_params = [2.0, 0.50, 0.143, 0.10]
    y_base, _ = simulate_plant(t, r, kp, ki, kd, base_params, y0)
    base_rmse = np.sqrt(np.mean((y_hw - y_base) ** 2))
    print(f"[*] Baseline Uncalibrated Model RMSE: {base_rmse:.3f} cm")

    # Objective function: minimize sum of squared tracking errors
    def objective(p):
        y_pred, _ = simulate_plant(t, r, kp, ki, kd, p, y0)
        return np.mean((y_hw - y_pred) ** 2)

    # Parameter bounds:
    # damping: [0.5, 4.0] N*s/cm
    # servo_alpha: [0.10, 0.80] (servo slew filter factor per sample)
    # linkage_ratio: [0.09, 0.20] deg beam / deg servo
    # coulomb: [0.0, 0.6] cm/s^2
    bounds = [
        (0.5, 4.0),
        (0.10, 0.80),
        (0.09, 0.20),
        (0.0, 0.60)
    ]

    print("[*] Running Non-Linear Least Squares Optimization (L-BFGS-B)...")
    res = minimize(objective, base_params, method='L-BFGS-B', bounds=bounds)

    opt_params = res.x
    y_opt, srv_opt = simulate_plant(t, r, kp, ki, kd, opt_params, y0)
    opt_rmse = np.sqrt(np.mean((y_hw - y_opt) ** 2))

    print("=" * 75)
    print("IDENTIFICATION RESULTS:")
    print(f"  • Viscous Damping (mu_visc)    : {opt_params[0]:.4f} N*s/cm (Base: {base_params[0]:.2f})")
    print(f"  • Servo Dynamic Factor (alpha) : {opt_params[1]:.4f}        (Base: {base_params[1]:.2f})")
    print(f"  • Kinematic Linkage Ratio      : {opt_params[2]:.4f} deg/deg (Base: {base_params[2]:.3f})")
    print(f"  • Coulomb Stiction Friction   : {opt_params[3]:.4f} cm/s^2 (Base: {base_params[3]:.2f})")
    print(f"  • Initial RMSE                 : {base_rmse:.3f} cm")
    print(f"  • Fitted Optimal RMSE          : {opt_rmse:.3f} cm ({(1.0 - opt_rmse/base_rmse)*100:.1f}% error reduction)")
    print("=" * 75)

    # Export identified parameters as JSON
    param_dict = {
        "plant_damping": round(float(opt_params[0]), 4),
        "servo_alpha": round(float(opt_params[1]), 4),
        "linkage_ratio": round(float(opt_params[2]), 4),
        "coulomb_friction": round(float(opt_params[3]), 4),
        "fit_rmse_cm": round(float(opt_rmse), 4),
        "source_log": csv_path
    }
    with open("identified_plant_params.json", "w") as f:
        json.dump(param_dict, f, indent=2)
    print("[*] Exported identified_plant_params.json")

    # Generate Publication-Quality Report Plot
    os.makedirs(os.path.dirname(output_plot), exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={'height_ratios': [2.5, 1]})

    ax1.set_title("System Identification: Physical Hardware Telemetry vs. Calibrated Model", fontsize=11, fontweight='bold', pad=10)
    ax1.plot(t, r, 'c--', label='Setpoint r(t)', linewidth=1.5)
    ax1.plot(t, y_hw, color='#f59e0b', label=f'Hardware Logged Echo (y_hw)', linewidth=2.0)
    ax1.plot(t, y_base, color='#94a3b8', linestyle=':', label=f'Baseline Model (RMSE: {base_rmse:.2f} cm)', linewidth=1.5)
    ax1.plot(t, y_opt, color='#10b981', label=f'Fitted Identified Model (RMSE: {opt_rmse:.2f} cm)', linewidth=2.0)
    ax1.set_ylabel("Position (cm)", fontsize=10)
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='lower right', framealpha=0.9)

    # Residuals plot
    residuals = y_hw - y_opt
    ax2.plot(t, residuals, color='#ef4444', linewidth=1.2, label=f'Model Residual Error (Mean: {np.mean(residuals):+.2f} cm)')
    ax2.axhline(0, color='gray', linestyle='--', linewidth=0.8)
    ax2.set_ylabel("Residual (cm)", fontsize=10)
    ax2.set_xlabel("Time (seconds)", fontsize=10)
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()
    plt.savefig(output_plot, dpi=300)
    plt.close()
    print(f"[*] Saved figure to: {output_plot}")

    return param_dict


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "logs/sample_hardware_step_test.csv"
    identify_system(path)

