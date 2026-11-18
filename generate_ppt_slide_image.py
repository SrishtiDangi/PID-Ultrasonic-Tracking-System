"""
generate_ppt_slide_image.py
===========================
Generates a presentation-ready benchmark plot and audit table card
specifically designed to fit into Slide 09's placeholder:
'[Insert benchmark plot / table from autotune_benchmark_results.json and controller_comparison_results.csv]'
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from analysis_suite import simulate_step_response

def generate_ppt_graphic():
    # 1. Simulate the key controller trajectories for step 15cm -> 25cm
    dt = 0.02
    t_final = 5.0
    t, y_p, _ = simulate_step_response(2.4, 0.0, 0.0, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_pd, _ = simulate_step_response(2.4, 0.0, 0.9, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_pi, _ = simulate_step_response(2.4, 0.04, 0.0, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_pid, _ = simulate_step_response(2.4, 0.04, 0.9, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_opt, _ = simulate_step_response(4.47, 0.005, 1.62, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    r = np.where(t < 0.5, 15.0, 25.0)

    # Set up high-res presentation graphic (12 x 7.5 inches, 300 DPI)
    fig = plt.figure(figsize=(11, 6.8), dpi=300)
    fig.patch.set_facecolor('#ffffff')

    # Subplot 1: Dynamic Transient Response Curves (Top 55%)
    ax1 = plt.axes([0.08, 0.44, 0.88, 0.50])
    ax1.set_facecolor('#f8fafc')
    for spine in ax1.spines.values():
        spine.set_color('#cbd5e1')
        spine.set_linewidth(1.0)

    # Grid
    ax1.grid(True, color='#e2e8f0', linestyle='--', linewidth=0.8, alpha=0.9)

    # Setpoint
    ax1.plot(t, r, color='#64748b', linestyle='--', linewidth=2.0, label='Target Setpoint r(t)', zorder=2)

    # Responses
    ax1.plot(t, y_p, color='#ef4444', linewidth=1.8, linestyle=':', label='P-Only (Kp=2.4) [Severe Oscillation]', zorder=3)
    ax1.plot(t, y_pi, color='#a855f7', linewidth=1.8, linestyle='-.', label='PI (Kp=2.4, Ki=0.04) [Undamped Ringing]', zorder=3)
    ax1.plot(t, y_pd, color='#f59e0b', linewidth=2.0, label='PD (Kp=2.4, Kd=0.9) [Fast, Steady Droop]', zorder=4)
    ax1.plot(t, y_pid, color='#0284c7', linewidth=2.5, label='Nominal PID (Kp=2.4, Ki=0.04, Kd=0.9) [Balanced]', zorder=5)
    ax1.plot(t, y_opt, color='#10b981', linewidth=2.8, label='ITAE Optimized PID (Kp=4.47, Ki=0.005, Kd=1.62)', zorder=6)

    # Labels and annotations
    ax1.set_title("CLOSED-LOOP TRANSIENT BENCHMARK // STEP DISTURBANCE (15cm → 25cm)", 
                  fontsize=11.5, fontweight='bold', color='#0f172a', pad=10)
    ax1.set_xlabel("Time (seconds)", fontsize=9.5, fontweight='bold', color='#475569')
    ax1.set_ylabel("Ball / Canopy Distance (cm)", fontsize=9.5, fontweight='bold', color='#475569')
    ax1.tick_params(colors='#475569', labelsize=8.5)
    ax1.set_xlim(0, 5.0)
    ax1.set_ylim(13.0, 27.5)

    # 2% settling band shading
    ax1.axhspan(24.8, 25.2, color='#10b981', alpha=0.15, label='±2% Target Settling Band', zorder=1)

    # Legend
    legend = ax1.legend(loc='lower right', frameon=True, facecolor='#ffffff', edgecolor='#cbd5e1', 
                        fontsize=8.0, labelcolor='#1e293b', framealpha=0.95)
    legend.get_frame().set_boxstyle('round,pad=0.4')

    # Subplot 2: Formatted Comparative Audit Table (Bottom 36%)
    ax2 = plt.axes([0.08, 0.04, 0.88, 0.32])
    ax2.axis('off')

    columns = [
        "Controller Configuration",
        "Gains [Kp, Ki, Kd]",
        "Rise Time (tr)",
        "Settling (ts 2%)",
        "Overshoot (Mp)",
        "ITAE Cost",
        "System Stability Status"
    ]

    table_data = [
        ["P-Only", "2.40 / 0.00 / 0.00", "0.73 s", "4.09 s (Slow)", "23.2 %", "13.55", "Marginally Stable (Poles on jω)"],
        ["PD Controller", "2.40 / 0.00 / 0.90", "1.13 s", "1.75 s", "0.7 %", "5.11", "Asymptotically Stable (Friction Droop)"],
        ["PI Controller", "2.40 / 0.04 / 0.00", "0.73 s", "4.01 s", "24.3 %", "14.61", "Routh-Hurwitz Instability Warning"],
        ["Nominal PID (Flagship)", "2.40 / 0.04 / 0.90", "1.11 s", "1.67 s", "1.8 %", "8.03", "★ Robust Closed-Loop Equilibrium"],
        ["Detuned Ziegler-Nichols", "1.91 / 0.06 / 2.22", "2.19 s", "4.50 s", "0.8 %", "13.50", "Conservative (Overdamped)"],
        ["ITAE Optimized PID", "4.47 / 0.005 / 1.62", "0.45 s", "1.45 s", "4.8 %", "3.27", "★ Minimum Settling Time & Energy"]
    ]

    col_colors = ["#e2e8f0"] * len(columns)
    table = ax2.table(cellText=table_data, colLabels=columns, cellLoc='center', loc='center',
                      colColours=["#e2e8f0"] * 7)

    table.auto_set_font_size(False)
    table.set_fontsize(8.2)
    table.scale(1.0, 1.45)

    # Style table cells
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#cbd5e1')
        cell.set_linewidth(0.8)
        if row == 0:
            cell.set_text_props(weight='bold', color='#0f172a')
            cell.set_facecolor('#e2e8f0')
        elif row in [4, 6]:  # Nominal PID and ITAE Optimized
            cell.set_facecolor('#ecfdf5')
            cell.set_text_props(weight='bold', color='#065f46')
        elif row == 1:       # P-only
            cell.set_text_props(color='#991b1b')
            cell.set_facecolor('#fff1f2')
        elif row == 3:       # PI
            cell.set_text_props(color='#86198f')
            cell.set_facecolor('#faf5ff')
        else:
            cell.set_facecolor('#ffffff')
            cell.set_text_props(color='#334155')

    # Output paths
    docs_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Docs", "ppt_benchmark_comparison_plot.png")
    root_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ppt_benchmark_comparison_plot.png")

    plt.savefig(docs_path, dpi=300, bbox_inches='tight', facecolor='#ffffff')
    plt.savefig(root_path, dpi=300, bbox_inches='tight', facecolor='#ffffff')
    plt.close()

    print(f"SUCCESS: Generated PPT Benchmark graphic at:\n  - {docs_path}\n  - {root_path}")

if __name__ == "__main__":
    generate_ppt_graphic()
