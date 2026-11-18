"""
generate_ppt_slide_image.py - Upgraded with Generous Margin Padding & Zero Edge Clipping
========================================================================================
Generates:
1. ppt_benchmark_comparison_plot.png (Combined Card with generous padding)
2. ppt_benchmark_plot_only.png (Clean Step Response Curve only)
3. ppt_benchmark_table_only.png (Clean Audit Table only)
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from analysis_suite import simulate_step_response

def generate_all_ppt_assets():
    # 1. Simulate key controller trajectories for step 15cm -> 25cm
    dt = 0.02
    t_final = 5.0
    t, y_p, _ = simulate_step_response(2.4, 0.0, 0.0, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_pd, _ = simulate_step_response(2.4, 0.0, 0.9, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_pi, _ = simulate_step_response(2.4, 0.04, 0.0, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_pid, _ = simulate_step_response(2.4, 0.04, 0.9, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    t, y_opt, _ = simulate_step_response(4.47, 0.005, 1.62, t_final=t_final, dt=dt, r_init=15.0, r_step=25.0)
    r = np.where(t < 0.5, 15.0, 25.0)

    # =========================================================================
    # 1. COMBINED GRAPHIC (PLOT + TABLE) WITH GENEROUS 40px SAFE MARGINS
    # =========================================================================
    fig = plt.figure(figsize=(10.5, 7.5), dpi=300)
    fig.patch.set_facecolor('#ffffff')

    # Draw a clean background card container with subtle rounded border
    card_ax = fig.add_axes([0, 0, 1, 1])
    card_ax.axis('off')
    card = patches.FancyBboxPatch((0.02, 0.02), 0.96, 0.96,
                                  boxstyle="round,pad=0.015,rounding_size=0.03",
                                  facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5)
    card_ax.add_patch(card)

    # Subplot 1: Response Plot (Safe margins: Left 0.09, Right 0.91)
    ax1 = fig.add_axes([0.10, 0.47, 0.80, 0.44])
    ax1.set_facecolor('#f8fafc')
    for spine in ax1.spines.values():
        spine.set_color('#cbd5e1')
        spine.set_linewidth(1.0)

    ax1.grid(True, color='#e2e8f0', linestyle='--', linewidth=0.8, alpha=0.9)

    # 2% settling band shading
    ax1.axhspan(24.8, 25.2, color='#10b981', alpha=0.15, label='±2% Target Band', zorder=1)

    # Setpoint
    ax1.plot(t, r, color='#64748b', linestyle='--', linewidth=2.0, label='Target r(t)', zorder=2)

    # Curves
    ax1.plot(t, y_p, color='#ef4444', linewidth=1.8, linestyle=':', label='P-Only (Kp=2.4) [Severe Ringing]', zorder=3)
    ax1.plot(t, y_pi, color='#a855f7', linewidth=1.8, linestyle='-.', label='PI (Kp=2.4, Ki=0.04) [RH Risk]', zorder=3)
    ax1.plot(t, y_pd, color='#f59e0b', linewidth=2.0, label='PD (Kp=2.4, Kd=0.9) [Fast, Offset]', zorder=4)
    ax1.plot(t, y_pid, color='#0284c7', linewidth=2.5, label='Nominal PID [Balanced]', zorder=5)
    ax1.plot(t, y_opt, color='#10b981', linewidth=2.8, label='ITAE Optimized PID (ts=1.45s)', zorder=6)

    ax1.set_title("CLOSED-LOOP TRANSIENT BENCHMARK (15cm → 25cm)", 
                  fontsize=11, fontweight='bold', color='#0f172a', pad=8)
    ax1.set_xlabel("Time (seconds)", fontsize=9, fontweight='bold', color='#475569')
    ax1.set_ylabel("Distance (cm)", fontsize=9, fontweight='bold', color='#475569')
    ax1.tick_params(colors='#475569', labelsize=8)
    ax1.set_xlim(0, 5.0)
    ax1.set_ylim(13.0, 27.5)

    legend = ax1.legend(loc='lower right', frameon=True, facecolor='#ffffff', edgecolor='#cbd5e1', 
                        fontsize=7.5, labelcolor='#1e293b', framealpha=0.95)
    legend.get_frame().set_boxstyle('round,pad=0.3')

    # Subplot 2: Table (Explicit Column Widths, perfectly within margins)
    ax2 = fig.add_axes([0.08, 0.05, 0.84, 0.35])
    ax2.axis('off')

    columns = [
        "Controller",
        "Gains [Kp, Ki, Kd]",
        "Rise (tr)",
        "Settling (ts)",
        "Overshoot",
        "ITAE",
        "Stability Status"
    ]

    table_data = [
        ["P-Only", "2.40 / 0.00 / 0.00", "0.73 s", "4.09 s", "23.2 %", "13.55", "Marginal (jω Poles)"],
        ["PD Controller", "2.40 / 0.00 / 0.90", "1.13 s", "1.75 s", "0.7 %", "5.11", "Stable (Friction Droop)"],
        ["PI Controller", "2.40 / 0.04 / 0.00", "0.73 s", "4.01 s", "24.3 %", "14.61", "Routh-Hurwitz Risk"],
        ["Nominal PID", "2.40 / 0.04 / 0.90", "1.11 s", "1.67 s", "1.8 %", "8.03", "★ Stable Equilibrium"],
        ["Detuned ZN", "1.91 / 0.06 / 2.22", "2.19 s", "4.50 s", "0.8 %", "13.50", "Overdamped"],
        ["ITAE Optimized", "4.47 / 0.005 / 1.62", "0.45 s", "1.45 s", "4.8 %", "3.27", "★ Minimum Settling Time"]
    ]

    # Explicit column widths that sum to 1.00
    col_widths = [0.18, 0.22, 0.10, 0.11, 0.11, 0.09, 0.19]
    table = ax2.table(cellText=table_data, colLabels=columns, colWidths=col_widths, 
                      cellLoc='center', loc='center')

    table.auto_set_font_size(False)
    table.set_fontsize(7.8)
    table.scale(1.0, 1.35)

    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#cbd5e1')
        cell.set_linewidth(0.8)
        if row == 0:
            cell.set_text_props(weight='bold', color='#0f172a')
            cell.set_facecolor('#e2e8f0')
        elif row in [4, 6]:
            cell.set_facecolor('#ecfdf5')
            cell.set_text_props(weight='bold', color='#065f46')
        elif row == 1:
            cell.set_text_props(color='#991b1b')
            cell.set_facecolor('#fff1f2')
        elif row == 3:
            cell.set_text_props(color='#86198f')
            cell.set_facecolor('#faf5ff')
        else:
            cell.set_facecolor('#ffffff')
            cell.set_text_props(color='#334155')

    # Save with generous padding
    combined_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ppt_benchmark_comparison_plot.png")
    docs_combined_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Docs", "ppt_benchmark_comparison_plot.png")
    fig.savefig(combined_path, dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    fig.savefig(docs_combined_path, dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    plt.close(fig)

    # =========================================================================
    # 2. INDIVIDUAL ASSET A: PLOT ONLY (Full width, no table clipping)
    # =========================================================================
    fig_plot, ax_p = plt.subplots(figsize=(9, 4.8), dpi=300)
    fig_plot.patch.set_facecolor('#ffffff')
    ax_p.set_facecolor('#f8fafc')
    for spine in ax_p.spines.values():
        spine.set_color('#cbd5e1')

    ax_p.grid(True, color='#e2e8f0', linestyle='--', linewidth=0.8)
    ax_p.axhspan(24.8, 25.2, color='#10b981', alpha=0.15, label='±2% Target Band', zorder=1)
    ax_p.plot(t, r, color='#64748b', linestyle='--', linewidth=2.0, label='Target r(t)', zorder=2)
    ax_p.plot(t, y_p, color='#ef4444', linewidth=1.8, linestyle=':', label='P-Only (Kp=2.4) [Severe Ringing]', zorder=3)
    ax_p.plot(t, y_pi, color='#a855f7', linewidth=1.8, linestyle='-.', label='PI (Kp=2.4, Ki=0.04) [RH Risk]', zorder=3)
    ax_p.plot(t, y_pd, color='#f59e0b', linewidth=2.0, label='PD (Kp=2.4, Kd=0.9) [Fast, Offset]', zorder=4)
    ax_p.plot(t, y_pid, color='#0284c7', linewidth=2.5, label='Nominal PID (Kp=2.4, Ki=0.04, Kd=0.9)', zorder=5)
    ax_p.plot(t, y_opt, color='#10b981', linewidth=2.8, label='ITAE Optimized PID (ts=1.45s)', zorder=6)

    ax_p.set_title("CLOSED-LOOP TRANSIENT BENCHMARK // STEP (15cm → 25cm)", 
                   fontsize=11.5, fontweight='bold', color='#0f172a', pad=10)
    ax_p.set_xlabel("Time (seconds)", fontsize=9.5, fontweight='bold', color='#475569')
    ax_p.set_ylabel("Distance (cm)", fontsize=9.5, fontweight='bold', color='#475569')
    ax_p.tick_params(colors='#475569', labelsize=8.5)
    ax_p.set_xlim(0, 5.0)
    ax_p.set_ylim(13.0, 27.5)
    ax_p.legend(loc='lower right', frameon=True, facecolor='#ffffff', edgecolor='#cbd5e1', fontsize=8.0)

    plot_only_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ppt_benchmark_plot_only.png")
    fig_plot.savefig(plot_only_path, dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    plt.close(fig_plot)

    # =========================================================================
    # 3. INDIVIDUAL ASSET B: TABLE ONLY (Full width card)
    # =========================================================================
    fig_tbl, ax_t = plt.subplots(figsize=(9.5, 3.2), dpi=300)
    fig_tbl.patch.set_facecolor('#ffffff')
    ax_t.axis('off')

    table_only = ax_t.table(cellText=table_data, colLabels=columns, colWidths=col_widths, 
                            cellLoc='center', loc='center')
    table_only.auto_set_font_size(False)
    table_only.set_fontsize(8.5)
    table_only.scale(1.0, 1.5)

    for (row, col), cell in table_only.get_celld().items():
        cell.set_edgecolor('#cbd5e1')
        cell.set_linewidth(0.8)
        if row == 0:
            cell.set_text_props(weight='bold', color='#0f172a')
            cell.set_facecolor('#e2e8f0')
        elif row in [4, 6]:
            cell.set_facecolor('#ecfdf5')
            cell.set_text_props(weight='bold', color='#065f46')
        elif row == 1:
            cell.set_text_props(color='#991b1b')
            cell.set_facecolor('#fff1f2')
        elif row == 3:
            cell.set_text_props(color='#86198f')
            cell.set_facecolor('#faf5ff')
        else:
            cell.set_facecolor('#ffffff')
            cell.set_text_props(color='#334155')

    table_only_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ppt_benchmark_table_only.png")
    fig_tbl.savefig(table_only_path, dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    plt.close(fig_tbl)

    print("SUCCESS: Generated 3 PPT assets with generous side margins and padding:")
    print(f"  1. Combined: {combined_path}")
    print(f"  2. Plot Only: {plot_only_path}")
    print(f"  3. Table Only: {table_only_path}")

if __name__ == "__main__":
    generate_all_ppt_assets()
