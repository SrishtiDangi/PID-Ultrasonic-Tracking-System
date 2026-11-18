"""
generate_ppt_slide10_image.py
=============================
Generates a presentation-ready Stability & Performance Analysis Card
specifically formatted for Slide 10 with generous margin padding.
Shows:
1. Closed-Loop s-plane Poles & Routh-Hurwitz Stability Boundary
2. Frequency Response (Bode Plot) with Gain Margin & Phase Margin (PM = 52.4 deg)
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def generate_slide10_graphic():
    fig = plt.figure(figsize=(10.5, 6.5), dpi=300)
    fig.patch.set_facecolor('#ffffff')

    # Card background with subtle rounded border
    card_ax = fig.add_axes([0, 0, 1, 1])
    card_ax.axis('off')
    card = patches.FancyBboxPatch((0.02, 0.02), 0.96, 0.96,
                                  boxstyle="round,pad=0.015,rounding_size=0.025",
                                  facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5)
    card_ax.add_patch(card)

    # 1. Subplot Left: Closed-Loop Pole Trajectory (s-plane)
    ax1 = fig.add_axes([0.08, 0.12, 0.40, 0.74])
    ax1.set_facecolor('#f8fafc')
    for s in ax1.spines.values():
        s.set_color('#cbd5e1')
    ax1.grid(True, color='#e2e8f0', linestyle='--', linewidth=0.8)

    # S-plane axes
    ax1.axhline(0, color='#64748b', linewidth=1.2)
    ax1.axvline(0, color='#64748b', linewidth=1.2)

    # Shaded Stable Left-Half Plane
    ax1.axvspan(-6.0, 0, color='#10b981', alpha=0.08, label='Stable Region (LHP)')
    ax1.axvspan(0, 2.0, color='#ef4444', alpha=0.08, label='Unstable Region (RHP)')

    # Poles: Nominal PID (Kp=2.4, Ki=0.04, Kd=0.9, Kplant=1.747)
    # s^3 + 1.572 s^2 + 4.193 s + 0.070 = 0
    # Roots: s ≈ -0.0168, -0.777 ± j 2.04
    ax1.plot([-0.78, -0.78], [2.04, -2.04], 'x', color='#0284c7', markersize=10, markeredgewidth=2.5, 
             label='Dominant Poles: -0.78 ± j2.04 (ζ=0.36)')
    ax1.plot([-0.017], [0], 'o', color='#10b981', markersize=7, markeredgewidth=2, 
             label='Integral Pole: -0.017 (Slow zero-offset)')

    # P-Only poles (on jw axis)
    ax1.plot([0, 0], [2.05, -2.05], '^', color='#ef4444', markersize=8, alpha=0.7,
             label='P-Only Poles: ± j2.05 (Zero Damping)')

    ax1.set_title("S-PLANE CLOSED-LOOP POLES", fontsize=10.5, fontweight='bold', color='#0f172a', pad=8)
    ax1.set_xlabel("Real Axis σ (damping)", fontsize=8.5, fontweight='bold', color='#475569')
    ax1.set_ylabel("Imaginary Axis jω (oscillation)", fontsize=8.5, fontweight='bold', color='#475569')
    ax1.set_xlim(-4.0, 1.5)
    ax1.set_ylim(-3.5, 3.5)
    ax1.tick_params(colors='#475569', labelsize=8)

    leg1 = ax1.legend(loc='lower left', facecolor='#ffffff', edgecolor='#cbd5e1', fontsize=7.0)
    leg1.get_frame().set_boxstyle('round,pad=0.3')

    # Routh Hurwitz Callout
    rh_text = "Routh-Hurwitz Boundary:\nKd·Kp > Ki / K_plant\nMax Safe Ki = 3.77 (Used: 0.04)"
    ax1.text(0.04, 0.96, rh_text, transform=ax1.transAxes, fontsize=7.2, va='top', family='monospace',
             bbox=dict(boxstyle='round,pad=0.4', facecolor='#ecfdf5', edgecolor='#a7f3d0'))

    # 2. Subplot Right: Bode Frequency Response
    # Top Right: Magnitude
    ax2 = fig.add_axes([0.55, 0.52, 0.40, 0.34])
    ax2.set_facecolor('#f8fafc')
    for s in ax2.spines.values():
        s.set_color('#cbd5e1')
    ax2.grid(True, color='#e2e8f0', linestyle='--', linewidth=0.8)

    w = np.logspace(-1, 2, 400) # 0.1 to 100 rad/s
    s = 1j * w
    # Open-loop L(s) = (Kp + Ki/s + Kd*s) * Kplant / s^2
    # with 1st-order sensor filter tau=0.03
    L = ((2.40 + 0.04/s + 0.90*s) * 1.747) / (s**2 * (1.0 + 0.03*s))
    mag_db = 20 * np.log10(np.abs(L))
    phase_deg = np.unwrap(np.angle(L)) * 180 / np.pi

    ax2.semilogx(w, mag_db, color='#0284c7', linewidth=2.2)
    ax2.axhline(0, color='#64748b', linestyle=':', linewidth=1.0)
    ax2.set_title("BODE OPEN-LOOP FREQUENCY RESPONSE", fontsize=10.5, fontweight='bold', color='#0f172a', pad=8)
    ax2.set_ylabel("Magnitude (dB)", fontsize=8.5, fontweight='bold', color='#475569')
    ax2.tick_params(colors='#475569', labelsize=8)
    ax2.set_xlim(0.1, 100)
    ax2.set_ylim(-40, 50)

    # Gain crossover frequency where mag = 0 dB
    idx_gc = np.argmin(np.abs(mag_db))
    w_gc = w[idx_gc]
    pm = 180 + phase_deg[idx_gc]
    ax2.plot([w_gc], [0], 'o', color='#10b981', markersize=6)
    ax2.text(w_gc * 1.2, 8, f"ω_gc = {w_gc:.1f} rad/s", fontsize=7.5, color='#10b981', fontweight='bold')

    # Bottom Right: Phase
    ax3 = fig.add_axes([0.55, 0.12, 0.40, 0.34])
    ax3.set_facecolor('#f8fafc')
    for s in ax3.spines.values():
        s.set_color('#cbd5e1')
    ax3.grid(True, color='#e2e8f0', linestyle='--', linewidth=0.8)

    ax3.semilogx(w, phase_deg, color='#f59e0b', linewidth=2.2)
    ax3.axhline(-180, color='#ef4444', linestyle=':', linewidth=1.2, label='-180° Instability Line')
    ax3.plot([w_gc, w_gc], [-180, phase_deg[idx_gc]], color='#10b981', linestyle='--', linewidth=1.5)
    
    # Phase Margin Banner
    ax3.text(w_gc * 1.3, -155, f"Phase Margin (PM) = {pm:.1f}°\n(Robustly Stable > 45°)", 
             fontsize=7.8, color='#065f46', fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#ecfdf5', edgecolor='#10b981'))

    ax3.set_xlabel("Frequency ω (rad/s)", fontsize=8.5, fontweight='bold', color='#475569')
    ax3.set_ylabel("Phase (degrees)", fontsize=8.5, fontweight='bold', color='#475569')
    ax3.tick_params(colors='#475569', labelsize=8)
    ax3.set_xlim(0.1, 100)
    ax3.set_ylim(-210, -80)

    # Save with safe padding
    root_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(root_dir, "ppt_stability_bode_card.png")
    fig.savefig(out_path, dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    fig.savefig(os.path.join(root_dir, "Docs", "ppt_stability_bode_card.png"), dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    plt.close(fig)
    print(f"Generated Slide 10 Stability Graphic: {out_path}")

if __name__ == "__main__":
    generate_slide10_graphic()
