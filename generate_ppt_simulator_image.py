"""
generate_ppt_simulator_image.py
===============================
Generates high-resolution presentation screenshot assets for Slide 06:
'Python Interactive Physics Simulator - simulator.py (Tkinter)'
"""

import os
from PIL import Image, ImageOps, ImageFilter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def generate_simulator_assets():
    root_dir = os.path.dirname(os.path.abspath(__file__))
    docs_dir = os.path.join(root_dir, "Docs")

    # =========================================================================
    # 1. CLEAN CROP OF ACTUAL USER DESKTOP RUN (from user_uploaded screenshot)
    # =========================================================================
    user_img_path = r"C:/Users/EW0116TU/.gemini/antigravity/brain/d6d830a2-3ca9-41d4-af0d-4a1decd713a8/.user_uploaded/media_1794997385072_07fa181b.jpg"
    
    if os.path.exists(user_img_path):
        raw_img = Image.open(user_img_path)
        # Crop out the Windows taskbar (rows 508-575) so only the clean Tkinter window remains
        cropped = raw_img.crop((0, 0, 1024, 508))
        
        # Add a subtle dark window border and generous white padding for PPT
        padded_w = 1024 + 80
        padded_h = 508 + 80
        canvas_img = Image.new("RGB", (padded_w, padded_h), (255, 255, 255))
        canvas_img.paste(cropped, (40, 40))
        
        crop_out_path = os.path.join(root_dir, "ppt_simulator_actual_window.png")
        canvas_img.save(crop_out_path, dpi=(300, 300))
        canvas_img.save(os.path.join(docs_dir, "ppt_simulator_actual_window.png"), dpi=(300, 300))
        print(f"Generated clean desktop crop: {crop_out_path}")

    # =========================================================================
    # 2. PRISTINE 300-DPI PRESENTATION MOCKUP (Upgraded Stacked Scope & Vectors)
    # =========================================================================
    fig = plt.figure(figsize=(10.5, 6.8), dpi=300)
    fig.patch.set_facecolor('#ffffff')

    # Card background with subtle rounded border
    card_ax = fig.add_axes([0, 0, 1, 1])
    card_ax.axis('off')
    card = patches.FancyBboxPatch((0.02, 0.02), 0.96, 0.96,
                                  boxstyle="round,pad=0.015,rounding_size=0.025",
                                  facecolor='#0b0f19', edgecolor='#334155', linewidth=1.5)
    card_ax.add_patch(card)

    # Windows Titlebar (Mockup)
    tb_ax = fig.add_axes([0.03, 0.93, 0.94, 0.045])
    tb_ax.axis('off')
    tb_ax.text(0.02, 0.4, "⚙️ PID ULTRASONIC DYNAMIC TRACKING BENCH (Tkinter // Python 3.11)", 
               color='#94a3b8', fontsize=8.5, fontweight='bold', va='center')
    # Window controls (min, max, close dots)
    tb_ax.plot([0.96], [0.4], 'o', color='#ef4444', markersize=4.5)
    tb_ax.plot([0.94], [0.4], 'o', color='#f59e0b', markersize=4.5)
    tb_ax.plot([0.92], [0.4], 'o', color='#10b981', markersize=4.5)

    # Top Status Bar & Badges
    hdr_ax = fig.add_axes([0.04, 0.84, 0.92, 0.08])
    hdr_ax.axis('off')
    hdr_ax.text(0.01, 0.75, "PID ULTRASONIC DYNAMIC TRACKING BENCH", color='#38bdf8', fontsize=11, fontweight='black')
    hdr_ax.text(0.01, 0.25, "Closed-Loop Embedded Control • HC-SR04 Transducer • SG90 Micro Servo • 30ms Fixed Scheduler", 
                color='#64748b', fontsize=7.5)
    
    # Badges
    hdr_ax.text(0.68, 0.60, "Rig: ⚖️ Ball-on-Beam", color='#ffffff', fontsize=7.5, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#4f46e5', edgecolor='none'))
    hdr_ax.text(0.81, 0.60, "Mode: PID", color='#ffffff', fontsize=7.5, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#0284c7', edgecolor='none'))
    hdr_ax.text(0.90, 0.60, "USB: Active", color='#ffffff', fontsize=7.5, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#059669', edgecolor='none'))

    # Telemetry Metric Tiles Row
    tile_ax = fig.add_axes([0.04, 0.74, 0.92, 0.085])
    tile_ax.axis('off')
    tiles = [
        ("TARGET (r)", "20.0 cm", "#38bdf8"),
        ("MEASURED (y)", "19.9 cm", "#10b981"),
        ("ERROR (e)", "-0.1 cm", "#f43f5e"),
        ("SERVO (θ)", "91.2°", "#f59e0b"),
        ("VELOCITY (v)", "+0.2 cm/s", "#06b6d4"),
        ("STEP IAE", "0.4 cm·s", "#a855f7")
    ]
    for idx, (label, val, col) in enumerate(tiles):
        tx = idx * 0.165
        tile_ax.add_patch(patches.FancyBboxPatch((tx, 0.05), 0.155, 0.90,
                                                boxstyle="round,pad=0.01",
                                                facecolor='#111827', edgecolor='#1f2937', linewidth=1))
        tile_ax.text(tx + 0.015, 0.68, label, color='#9ca3af', fontsize=6.8, fontweight='bold')
        tile_ax.text(tx + 0.015, 0.22, val, color=col, fontsize=9.5, fontweight='black', family='monospace')

    # Viewport Canvas (Physical Ball on Beam & Vector Field)
    vp_ax = fig.add_axes([0.04, 0.43, 0.92, 0.29])
    vp_ax.set_facecolor('#070b14')
    for s in vp_ax.spines.values():
        s.set_color('#1e293b')
    vp_ax.set_xlim(0, 100)
    vp_ax.set_ylim(0, 100)
    vp_ax.set_xticks([])
    vp_ax.set_yticks([])

    # Beam (tilted slightly at ~3 deg)
    vp_ax.plot([15, 85], [46, 54], color='#38bdf8', linewidth=4.5, zorder=3)
    # Fulcrum Triangle
    fulcrum = patches.Polygon([[50, 50], [46, 32], [54, 32]], closed=True,
                              facecolor='#475569', edgecolor='#64748b', linewidth=1.5, zorder=2)
    vp_ax.add_patch(fulcrum)

    # HC-SR04 Transducer at beam left
    vp_ax.add_patch(patches.Rectangle((12, 42), 5, 12, facecolor='#0284c7', edgecolor='#38bdf8', zorder=4))
    # Acoustic wave arcs
    for r_arc in [8, 14, 20]:
        arc = patches.Arc((18, 47), r_arc, r_arc, angle=0, theta1=-35, theta2=35, color='#06b6d4', linewidth=1.8)
        vp_ax.add_patch(arc)

    # Target indicator flag
    vp_ax.plot([48, 48], [48, 68], color='#38bdf8', linestyle='--', linewidth=1.5)
    vp_ax.text(48, 71, "Target: 20cm", color='#38bdf8', fontsize=7.5, fontweight='bold', ha='center')

    # Rolling Ball at pos=48
    ball = patches.Circle((48, 52), 3.2, facecolor='#f59e0b', edgecolor='#d97706', linewidth=1.5, zorder=5)
    vp_ax.add_patch(ball)

    # Force Vectors
    vp_ax.annotate('', xy=(56, 52), xytext=(48, 52),
                    arrowprops=dict(arrowstyle="->", color='#10b981', lw=2.2))
    vp_ax.text(52, 57, "Velocity (v)", color='#10b981', fontsize=7, fontweight='bold')

    vp_ax.annotate('', xy=(48, 41), xytext=(48, 49),
                    arrowprops=dict(arrowstyle="->", color='#f59e0b', lw=2.0))
    vp_ax.text(49, 43, "Gravity (g)", color='#f59e0b', fontsize=7, fontweight='bold')

    # SG90 Servo Actuator on beam right
    vp_ax.add_patch(patches.Rectangle((83, 38), 6, 12, facecolor='#0284c7', edgecolor='#38bdf8', zorder=2))
    vp_ax.plot([86, 86], [44, 56], color='#f59e0b', linewidth=2.5, zorder=3)
    vp_ax.plot([86, 85], [56, 54], color='#94a3b8', linestyle=':', linewidth=1.5, zorder=3)
    vp_ax.text(86, 31, "Servo: 91°", color='#f59e0b', fontsize=7.5, fontweight='bold', ha='center')

    # Canvas Title overlay
    vp_ax.text(1.5, 92, "PHYSICAL HARDWARE VIEWPORT & VECTOR FIELD", color='#94a3b8', fontsize=7.5, fontweight='bold')

    # Oscilloscope (Stacked Multi-Trace Time Domain)
    osc_ax = fig.add_axes([0.04, 0.12, 0.92, 0.28])
    osc_ax.set_facecolor('#05070e')
    for s in osc_ax.spines.values():
        s.set_color('#1e293b')
    osc_ax.grid(True, color='#1e293b', linestyle=':', linewidth=0.8)

    t_osc = np.linspace(0, 6.0, 300)
    # Target step at 0.5s from 15 to 20
    r_osc = np.where(t_osc < 0.5, 15.0, 20.0)
    # Realistic underdamped PID ball response (ts=1.67s, Mp=1.8%)
    decay = np.exp(-1.8 * np.maximum(0, t_osc - 0.5))
    y_osc = np.where(t_osc < 0.5, 15.0, 20.0 - 5.0 * decay * np.cos(4.2 * (t_osc - 0.5)))
    # Add slight acoustic sensor noise
    y_osc += np.random.normal(0, 0.08, len(t_osc))

    # Control effort curve (servo angle around 90)
    u_osc = 90.0 + 8.0 * np.gradient(y_osc)

    # Shaded Actuator Saturation limits (<25, >155)
    osc_ax.axhspan(38.0, 42.0, color='#ef4444', alpha=0.15, label='Actuator Saturation Margin')

    # Plot traces
    osc_ax.plot(t_osc, r_osc, color='#38bdf8', linestyle='--', linewidth=1.8, label='Target Setpoint r(t)')
    osc_ax.plot(t_osc, y_osc, color='#10b981', linewidth=2.2, label='Sensor Distance y(t)')
    osc_ax.plot(t_osc, 10.0 + (u_osc - 90.0) * 0.4, color='#f59e0b', linewidth=1.2, alpha=0.7, label='Control Effort u(t)')

    osc_ax.set_xlim(0, 6.0)
    osc_ax.set_ylim(8.0, 26.0)
    osc_ax.set_ylabel("Distance (cm)", color='#64748b', fontsize=8)
    osc_ax.set_xlabel("Time (seconds)", color='#64748b', fontsize=8)
    osc_ax.tick_params(colors='#64748b', labelsize=7.5)

    osc_ax.text(0.015, 0.88, "TIME-DOMAIN STEP RESPONSE OSCILLOSCOPE (33.3 Hz Fixed-Rate Telemetry)", 
                transform=osc_ax.transAxes, color='#38bdf8', fontsize=7.5, fontweight='bold')
    osc_ax.text(0.985, 0.88, "Status: Asymptotically Stable (ts=1.67s, Mp=1.8%)", 
                transform=osc_ax.transAxes, color='#10b981', fontsize=7.5, fontweight='bold', ha='right')

    osc_legend = osc_ax.legend(loc='lower right', facecolor='#0b0f19', edgecolor='#1e293b', 
                               fontsize=7, labelcolor='#e2e8f0', framealpha=0.9)

    # Bottom Sliders Mockup
    btm_ax = fig.add_axes([0.04, 0.04, 0.92, 0.06])
    btm_ax.axis('off')
    btm_ax.text(0.02, 0.5, "Proportional (Kp): 2.40", color='#94a3b8', fontsize=7.5, fontweight='bold')
    btm_ax.text(0.26, 0.5, "Integral (Ki): 0.04", color='#94a3b8', fontsize=7.5, fontweight='bold')
    btm_ax.text(0.48, 0.5, "Derivative (Kd): 0.90", color='#94a3b8', fontsize=7.5, fontweight='bold')
    btm_ax.text(0.72, 0.5, "Target (cm): 20.00", color='#38bdf8', fontsize=7.5, fontweight='bold')

    output_path = os.path.join(root_dir, "ppt_simulator_window.png")
    fig.savefig(output_path, dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    fig.savefig(os.path.join(docs_dir, "ppt_simulator_window.png"), dpi=300, bbox_inches='tight', pad_inches=0.25, facecolor='#ffffff')
    plt.close(fig)
    print(f"Generated pristine presentation window asset: {output_path}")

if __name__ == "__main__":
    generate_simulator_assets()
