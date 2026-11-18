"""
================================================================================
PID ULTRASONIC TRACKING SYSTEM - ADVANCED ANALYSIS SUITE (PARTS B1, B2, B3)
Features:
- Live Transient Metrics Engine (B1: tr, ts, Mp, ess, ITAE, RMSE, sine phase lag)
- 4-Way Controller Comparison (B2: P, PD, PI, PID overlay, CSV & PNG export)
- Stability Analysis Engine (B3: Poles, Bode GM/PM, Routh-Hurwitz boundary)
================================================================================
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from test_transient_metrics import compute_transient_metrics


# ==============================================================================
# B1: TRANSIENT & HARMONIC METRICS ENGINE
# ==============================================================================

def compute_sine_tracking_lag(t, r, y):
    """
    Computes phase delay / tracking lag between reference sine and feedback in ms.
    Uses cross-correlation peak finding.
    """
    if len(t) < 50:
        return 0.0

    dt = np.mean(np.diff(t))
    r_zero = r - np.mean(r)
    y_zero = y - np.mean(y)

    # Cross correlation
    corr = np.correlate(r_zero, y_zero, mode='full')
    lags = np.arange(-len(r_zero) + 1, len(r_zero))

    max_idx = np.argmax(corr)
    lag_samples = lags[max_idx]
    lag_ms = float(lag_samples * dt * 1000.0)

    return lag_ms


# ==============================================================================
# B2: 4-WAY CONTROLLER COMPARATOR (P vs PD vs PI vs PID)
# ==============================================================================

def simulate_step_response(kp, ki, kd, t_final=8.0, dt=0.01, r_init=15.0, r_step=30.0):
    """
    Simulates the closed-loop ball-on-beam response for arbitrary gains.
    """
    t = np.arange(0.0, t_final, dt)
    N = len(t)
    y = np.zeros(N)
    u_hist = np.zeros(N)

    pos = r_init
    vel = 0.0
    integral = 0.0
    prev_err = 0.0
    filt_d = 0.0
    servo = 90.0

    # Plant constants
    K_plant = 1.747 # cm/(s^2 * deg)
    damping = 1.80  # viscous friction

    for i in range(N):
        r = r_init if t[i] < 0.5 else r_step
        err = r - pos
        integral += err * dt
        integral = np.clip(integral, -15.0, 15.0)

        raw_d = (err - prev_err) / dt if dt > 0 else 0.0
        filt_d = 0.20 * raw_d + 0.80 * filt_d
        prev_err = err

        u = kp * err + ki * integral + kd * filt_d
        target_servo = np.clip(90.0 + u, 25.0, 155.0)
        servo += 0.35 * (target_servo - servo) # actuator lag

        # Beam tilt and acceleration
        beam_deg = 0.143 * (servo - 90.0)
        acc = (12.23 * beam_deg) - (damping * vel)

        vel += acc * dt
        pos += vel * dt
        pos = np.clip(pos, 2.0, 48.0)

        y[i] = pos
        u_hist[i] = servo

    return t, y, u_hist


def run_controller_comparison(output_dir="Docs"):
    """
    Runs benchmark comparison of P, PD, PI, and PID on a 15cm -> 30cm step.
    Saves CSV results and comparison plot PNG.
    """
    os.makedirs(output_dir, exist_ok=True)

    configs = {
        "P-Only": (2.40, 0.00, 0.00, '#ef4444'),
        "PD":     (2.40, 0.00, 0.90, '#f59e0b'),
        "PI":     (2.40, 0.04, 0.00, '#8b5cf6'),
        "PID":    (2.40, 0.04, 0.90, '#10b981')
    }

    results = []
    t_final = 7.0
    dt = 0.01
    r_init = 15.0
    r_step = 30.0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={'height_ratios': [2.5, 1.2]})
    ax1.set_title("Controller Comparison: Step Response Across Topologies (P vs PD vs PI vs PID)", fontsize=11, fontweight='bold')

    ref_trace = np.where(np.arange(0.0, t_final, dt) < 0.5, r_init, r_step)
    t_axis = np.arange(0.0, t_final, dt)
    ax1.plot(t_axis, ref_trace, 'c--', label='Setpoint r(t) [15cm → 30cm]', linewidth=1.5)

    for name, (kp, ki, kd, color) in configs.items():
        t, y, u = simulate_step_response(kp, ki, kd, t_final, dt, r_init, r_step)

        # Slice after step transition (t >= 0.5) for metrics
        mask = t >= 0.5
        t_sub = t[mask] - 0.5
        y_sub = y[mask]
        m = compute_transient_metrics(t_sub, y_sub, r_initial=r_init, r_final=r_step)

        # Check stability based on tail variance
        tail = y[-int(len(y)*0.2):]
        is_stable = np.std(tail) < 1.0

        results.append({
            "Controller": name,
            "Kp": kp,
            "Ki": ki,
            "Kd": kd,
            "RiseTime_s": round(m['rise_time'], 3) if m['rise_time'] else "N/A",
            "SettlingTime_s": round(m['settling_time'], 3) if m['settling_time'] else "> 7.0s",
            "Overshoot_pct": round(m['overshoot_pct'], 1),
            "SteadyStateErr_cm": round(m['steady_state_error'], 3),
            "ITAE": round(m['itae'], 2),
            "RMSE_cm": round(m['rmse'], 3),
            "Status": "STABLE" if is_stable else "UNSTABLE / LIMIT CYCLE"
        })

        label_txt = f"{name} (ts={m['settling_time'] or '>7'}s, Mp={m['overshoot_pct']:.0f}%, ess={m['steady_state_error']:.1f}cm)"
        ax1.plot(t, y, label=label_txt, color=color, linewidth=1.8)
        ax2.plot(t, u, label=f"{name}", color=color, linewidth=1.2)

    ax1.set_ylabel("Position (cm)", fontsize=10)
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='lower right', framealpha=0.9, fontsize=8)

    ax2.set_ylabel("Servo Angle (°)", fontsize=10)
    ax2.set_xlabel("Time (seconds)", fontsize=10)
    ax2.axhline(90, color='gray', linestyle=':', label='Neutral 90°')
    ax2.axhspan(155, 180, color='#ef4444', alpha=0.2, label='Saturation Zone')
    ax2.axhspan(0, 25, color='#ef4444', alpha=0.2)
    ax2.set_ylim(20, 160)
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    plot_path = os.path.join(output_dir, "controller_comparison.png")
    plt.savefig(plot_path, dpi=300)
    plt.close()
    print(f"[*] Saved controller comparison plot: {plot_path}")

    # Export to CSV
    csv_path = "controller_comparison_results.csv"
    with open(csv_path, "w") as f:
        headers = list(results[0].keys())
        f.write(",".join(headers) + "\n")
        for r in results:
            f.write(",".join(str(r[h]) for h in headers) + "\n")
    print(f"[*] Exported metrics table: {csv_path}")

    # Print pretty summary table
    print("\n" + "=" * 90)
    print(f"{'Controller':<12} | {'Rise Time':<10} | {'Settling (2%)':<14} | {'Overshoot':<10} | {'ess (cm)':<10} | {'ITAE':<8} | {'Status'}")
    print("-" * 90)
    for r in results:
        ts_str = f"{r['SettlingTime_s']}s" if isinstance(r['SettlingTime_s'], float) else r['SettlingTime_s']
        tr_str = f"{r['RiseTime_s']}s" if isinstance(r['RiseTime_s'], float) else r['RiseTime_s']
        print(f"{r['Controller']:<12} | {tr_str:<10} | {ts_str:<14} | {r['Overshoot_pct']:<9}% | {r['SteadyStateErr_cm']:<10} | {r['ITAE']:<8} | {r['Status']}")
    print("=" * 90)

    return results


# ==============================================================================
# B3: STABILITY, BODE & ROUTH-HURWITZ ANALYZER
# ==============================================================================

def analyze_stability(kp=2.40, ki=0.04, kd=0.90, K_plant=1.747, output_dir="Docs"):
    """
    Routh-Hurwitz Stability Criterion & Bode Frequency Domain Analysis
    Characteristic Equation:
      s^3 + K_plant*Kd * s^2 + K_plant*Kp * s + K_plant*Ki = 0
    """
    os.makedirs(output_dir, exist_ok=True)

    a3 = 1.0
    a2 = K_plant * kd
    a1 = K_plant * kp
    a0 = K_plant * ki

    poly_coeffs = [a3, a2, a1, a0]
    poles = np.roots(poly_coeffs)

    # Routh Hurwitz check:
    # b1 = (a2 * a1 - a3 * a0) / a2
    b1 = (a2 * a1 - a3 * a0) / a2 if a2 > 0 else -1.0
    is_rh_stable = (a3 > 0) and (a2 > 0) and (a1 > 0) and (a0 > 0) and (b1 > 0)
    critical_ki = a2 * a1 / a3 # upper limit on Ki for stability

    print("\n" + "=" * 70)
    print("STABILITY & CLOSED-LOOP POLE-ZERO AUDIT (B3):")
    print(f"  • Plant Gain K_plant         : {K_plant:.4f} cm/(s^2 * deg)")
    print(f"  • Gains Tested               : Kp={kp:.2f}, Ki={ki:.4f}, Kd={kd:.2f}")
    print(f"  • Characteristic Polynomial  : s^3 + {a2:.3f} s^2 + {a1:.3f} s + {a0:.4f} = 0")
    print(f"  • Routh b1 Coefficient       : {b1:.4f} (Must be > 0)")
    print(f"  • Maximum Stable Ki Limit    : {critical_ki / K_plant:.4f}")
    print(f"  • Routh-Hurwitz Status       : {'STABLE' if is_rh_stable else 'UNSTABLE'}")
    print("  • Closed-Loop Poles:")
    for idx, p in enumerate(poles, 1):
        sign = "+" if p.imag >= 0 else "-"
        damping_ratio = -p.real / np.abs(p) if np.abs(p) > 0 else 0.0
        print(f"      p{idx} = {p.real:+.4f} {sign} {abs(p.imag):.4f}j  (zeta = {damping_ratio:.3f})")
    print("=" * 70)

    # Frequency Domain / Bode Plot
    w = np.logspace(-2, 2, 500) # 0.01 to 100 rad/s
    s = 1j * w

    # Controller C(s) = Kd*s + Kp + Ki/s
    C_s = (kd * s**2 + kp * s + ki) / s
    # Plant G(s) = K_plant / s^2
    G_s = K_plant / (s**2)

    L_s = C_s * G_s # Open-loop transfer function

    mag = np.abs(L_s)
    mag_db = 20.0 * np.log10(mag)
    phase_deg = np.unwrap(np.angle(L_s)) * 180.0 / np.pi

    # Gain Margin & Phase Margin
    # Phase Crossover Frequency w_pc: where phase = -180 deg
    idx_pc = np.argmin(np.abs(phase_deg - (-180.0)))
    w_pc = w[idx_pc]
    gain_margin_db = -mag_db[idx_pc]

    # Gain Crossover Frequency w_gc: where mag_db = 0 dB
    idx_gc = np.argmin(np.abs(mag_db - 0.0))
    w_gc = w[idx_gc]
    phase_margin_deg = 180.0 + phase_deg[idx_gc]

    print(f"  • Gain Crossover Freq (w_gc) : {w_gc:.3f} rad/s")
    print(f"  • Phase Margin (PM)          : {phase_margin_deg:.1f}°")
    print(f"  • Phase Crossover Freq (w_pc): {w_pc:.3f} rad/s")
    print(f"  • Gain Margin (GM)           : {gain_margin_db:.1f} dB")
    print("=" * 70)

    # Plot Bode & Poles
    fig = plt.figure(figsize=(11, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.8, 1.2])

    ax_mag = fig.add_subplot(gs[0, 0])
    ax_phase = fig.add_subplot(gs[1, 0], sharex=ax_mag)
    ax_poles = fig.add_subplot(gs[:, 1])

    # Magnitude
    ax_mag.set_title("Open-Loop Frequency Response (Bode Plot)", fontsize=10, fontweight='bold')
    ax_mag.semilogx(w, mag_db, color='#38bdf8', linewidth=1.8)
    ax_mag.axhline(0, color='gray', linestyle='--', linewidth=0.8)
    ax_mag.set_ylabel("Magnitude (dB)", fontsize=9)
    ax_mag.grid(True, which='both', linestyle='--', alpha=0.5)

    # Phase
    ax_phase.semilogx(w, phase_deg, color='#fbbf24', linewidth=1.8)
    ax_phase.axhline(-180, color='#ef4444', linestyle='--', linewidth=0.8, label='-180° Critical')
    ax_phase.set_ylabel("Phase (degrees)", fontsize=9)
    ax_phase.set_xlabel("Frequency (rad/s)", fontsize=9)
    ax_phase.grid(True, which='both', linestyle='--', alpha=0.5)
    ax_phase.legend(loc='lower left', fontsize=8)

    # Closed-loop Pole-Zero map
    ax_poles.set_title("Closed-Loop Complex s-Plane Poles", fontsize=10, fontweight='bold')
    ax_poles.axvline(0, color='#ef4444', linestyle='--', linewidth=1.2, label='Imag Axis (Stability Bound)')
    ax_poles.axhline(0, color='gray', linestyle=':', linewidth=0.8)
    ax_poles.scatter(poles.real, poles.imag, marker='x', s=100, color='#10b981' if is_rh_stable else '#ef4444', linewidth=2.5, label='Closed-Loop Poles')

    for p in poles:
        ax_poles.annotate(f"{p.real:.2f}\n{p.imag:+.2f}j", (p.real, p.imag), textcoords="offset points", xytext=(8, -4), fontsize=8, color='#cbd5e1')

    ax_poles.set_xlabel("Real Axis (sigma)", fontsize=9)
    ax_poles.set_ylabel("Imag Axis (j*omega)", fontsize=9)
    ax_poles.grid(True, linestyle='--', alpha=0.5)
    ax_poles.legend(loc='upper right', fontsize=8)

    # Shaded stable Left Half Plane
    x_min, x_max = ax_poles.get_xlim()
    if x_min < 0:
        ax_poles.axvspan(x_min, 0, color='#10b981', alpha=0.08, label='Stable LHP')

    plt.tight_layout()
    plot_file = os.path.join(output_dir, "stability_bode_poles.png")
    plt.savefig(plot_file, dpi=300)
    plt.close()
    print(f"[*] Saved Bode and Pole-Zero plot: {plot_file}")

    return {
        "is_stable": is_rh_stable,
        "poles": [complex(p) for p in poles],
        "gain_margin_db": gain_margin_db,
        "phase_margin_deg": phase_margin_deg,
        "w_gc": w_gc,
        "w_pc": w_pc
    }


if __name__ == "__main__":
    print("[*] Running Controller Comparison (B2)...")
    run_controller_comparison()

    print("\n[*] Running Stability and Frequency Domain Analysis (B3)...")
    analyze_stability()
