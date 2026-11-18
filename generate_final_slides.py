"""
generate_final_slides.py - Professional Edition with Cambria Typography & Pictorial Flowchart
============================================================================================
Generates 16:9 300-DPI slides:
- Slide 11: Cyber-Physical Architecture with Pictorial Flowchart & Equal Spacing
- Slide 12: Real-World Applications (Heavy on Industrial Relevance)
- Slide 13: Conclusion, Academic References & Defense Takeaways in Cambria Typography
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Check Cambria font
FONT_TITLE = 'Cambria'
FONT_BODY = 'Cambria'

def draw_canva_decorations(ax):
    """Draws subtle decorative circular accents matching the Canva template."""
    # Top-right corner circles
    ax.add_patch(patches.Circle((15.2, 8.2), 1.3, facecolor='#5c82a6', alpha=0.32, edgecolor='none'))
    ax.add_patch(patches.Circle((13.9, 8.7), 0.8, facecolor='#486b88', alpha=0.42, edgecolor='none'))
    ax.add_patch(patches.Circle((14.5, 6.9), 0.55, facecolor='none', edgecolor='#5c82a6', linewidth=2.0, alpha=0.55))
    ax.add_patch(patches.Circle((12.9, 8.0), 0.65, facecolor='none', edgecolor='#5c82a6', linewidth=1.5, alpha=0.45))

    # Bottom-right corner circles
    ax.add_patch(patches.Circle((15.1, 0.9), 1.5, facecolor='#5c82a6', alpha=0.40, edgecolor='none'))
    ax.add_patch(patches.Circle((13.6, 0.45), 0.75, facecolor='#486b88', alpha=0.28, edgecolor='none'))
    ax.add_patch(patches.Circle((14.1, 1.8), 0.65, facecolor='none', edgecolor='#5c82a6', linewidth=2.0, alpha=0.55))
    ax.add_patch(patches.Circle((12.6, 1.05), 0.85, facecolor='none', edgecolor='#5c82a6', linewidth=1.5, alpha=0.45))

    # Top-left subtle accent
    ax.add_patch(patches.Circle((0.8, 8.4), 0.9, facecolor='#5c82a6', alpha=0.22, edgecolor='none'))
    ax.add_patch(patches.Circle((1.7, 8.8), 0.45, facecolor='none', edgecolor='#5c82a6', linewidth=1.5, alpha=0.35))


def generate_slide_11(out_path):
    """Slide 11: Cyber-Physical Architecture with Pictorial Flowchart & Equal Spacing"""
    fig = plt.figure(figsize=(16, 9), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor('#faf7f2')
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis('off')

    draw_canva_decorations(ax)

    # Page number box bottom-left
    ax.add_patch(patches.Rectangle((1.0, 0.6), 0.75, 0.75, facecolor='#3f6588', edgecolor='none'))
    ax.text(1.375, 0.975, "11", color='#ffffff', fontsize=16, fontweight='bold', fontname=FONT_TITLE, ha='center', va='center')

    # Main Title & Subtitle with equal breathing room
    ax.text(1.0, 8.0, "Cyber-Physical Architecture & Key Results", 
            color='#1e3a5f', fontsize=28, fontweight='bold', fontname=FONT_TITLE)
    ax.text(1.0, 7.55, "End-to-End Pipeline: Telemetry Logging, System Identification & Web Digital Twin", 
            color='#5c768d', fontsize=12, fontweight='bold', fontname=FONT_BODY)

    # =========================================================================
    # PICTORIAL FLOW CHART CONTAINER (Equal spacing: X = 1.0 to 15.0, W = 14.0)
    # =========================================================================
    ax.add_patch(patches.FancyBboxPatch((1.0, 4.6), 14.0, 2.65,
                                        boxstyle="round,pad=0.15,rounding_size=0.18",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(1.3, 6.95, "PICTORIAL SYSTEM FLOWCHART: CLOSED-LOOP MODEL CALIBRATION", 
            color='#1e3a5f', fontsize=11.5, fontweight='bold', fontname=FONT_TITLE)

    # 4 Flowchart Stage Blocks with pictorial sub-cards
    stage_w = 3.1
    spacing = 0.45
    start_x = 1.3

    blocks = [
        ("STAGE 1: HARDWARE & SENSING",
         "Arduino Uno + HC-SR04 Transducer",
         "• 33.3 Hz fixed micros() loop\n• 5ms ultrasonic echo timeout\n• Stream timestamped CSV log\n  (Target, Echo, Servo Angle)",
         '#e0f2fe', '#0369a1'),

        ("STAGE 2: PARAMETER ESTIMATION",
         "SciPy L-BFGS-B Optimizer",
         "• Non-linear least squares fit\n• Evaluates model prediction\n• Fits viscous rolling friction\n• Identifies actuator servo lag",
         '#f0fdf4', '#15803d'),

        ("STAGE 3: CALIBRATED MODEL",
         "Identified Plant Parameters",
         "• Friction: b = 0.082 N·s/m\n• Servo Lag: τ = 65 ms\n• Linkage Ratio: 0.1429°/°\n• 30.6% RMSE Error Reduction",
         '#fef3c7', '#b45309'),

        ("STAGE 4: FLAGSHIP DIGITAL TWIN",
         "Browser Web Bench & Simulator",
         "• 60 FPS vector field kinematics\n• Real-time digital phosphor scope\n• Disturbance injection suite\n• Live Bode & pole stability audit",
         '#ede9fe', '#6d28d9')
    ]

    for i, (b_title, b_sub, b_body, bg_col, b_border) in enumerate(blocks):
        bx = start_x + i * (stage_w + spacing)
        # Block Card
        ax.add_patch(patches.FancyBboxPatch((bx, 4.8), stage_w, 1.95,
                                            boxstyle="round,pad=0.08,rounding_size=0.12",
                                            facecolor='#ffffff', edgecolor=b_border, linewidth=1.2))
        # Top banner of block
        ax.add_patch(patches.FancyBboxPatch((bx, 6.25), stage_w, 0.50,
                                            boxstyle="round,pad=0.06,rounding_size=0.08",
                                            facecolor=bg_col, edgecolor=b_border, linewidth=0.8))
        ax.text(bx + 0.12, 6.55, b_title, color='#0f172a', fontsize=8.8, fontweight='bold', fontname=FONT_TITLE)
        ax.text(bx + 0.12, 6.35, b_sub, color='#475569', fontsize=7.6, fontname=FONT_BODY, fontstyle='italic')
        ax.text(bx + 0.12, 4.95, b_body, color='#334155', fontsize=8.0, fontname=FONT_BODY, linespacing=1.3)

        # Connecting Pipeline Arrow
        if i < 3:
            arrow_x = bx + stage_w + 0.06
            ax.annotate('', xy=(arrow_x + 0.32, 5.75), xytext=(arrow_x, 5.75),
                        arrowprops=dict(arrowstyle="-|>", color='#64748b', lw=2.5, mutation_scale=12))

    # =========================================================================
    # BOTTOM TWO CARDS (Equal spacing & balanced geometry)
    # Total width = 14.0, spacing = 0.6 -> Each card width = 6.7
    # =========================================================================
    card_w = 6.7
    bottom_y = 1.6
    bottom_h = 2.75

    # Bottom Left Card: Key Measured Results
    ax.add_patch(patches.FancyBboxPatch((1.0, bottom_y), card_w, bottom_h,
                                        boxstyle="round,pad=0.12,rounding_size=0.18",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(1.3, bottom_y + bottom_h - 0.35, "Key Measured Results & Calibration Fit", 
            color='#1e3a5f', fontsize=13, fontweight='bold', fontname=FONT_TITLE)

    results_text = (
        "• Sim-to-Real Discrepancy Reduction:\n"
        "   Model RMSE cut from 2.14 cm → 1.48 cm (30.6% improvement over pure physics).\n\n"
        "• Physical Plant Parameters Identified from Logs:\n"
        "   - SG90 Servo Actuator Lag: τ(servo) ≈ 65 ms (first-order time constant)\n"
        "   - Beam Viscous Rolling Friction: b ≈ 0.082 N·s/m\n\n"
        "• Dynamic Settling Performance:\n"
        "   - Settling Time (ts 2%): 1.45 s (ITAE Optimizer) / 1.67 s (Nominal PID)\n"
        "   - Peak Overshoot (Mp): Under 5.0% (completely eliminating mechanical ringing)\n"
        "   - Steady-State Error (ess): < 0.05 cm (offset removed by anti-windup integral)"
    )
    ax.text(1.3, bottom_y + 0.25, results_text, color='#334155', fontsize=8.8, fontname=FONT_BODY, linespacing=1.32)

    # Bottom Right Card: Future Scope & Platform Extensions
    ax.add_patch(patches.FancyBboxPatch((8.3, bottom_y), card_w, bottom_h,
                                        boxstyle="round,pad=0.12,rounding_size=0.18",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(8.6, bottom_y + bottom_h - 0.35, "Future Scope & Platform Extensions", 
            color='#1e3a5f', fontsize=13, fontweight='bold', fontname=FONT_TITLE)

    future_text = (
        "1. 2-DOF Ball-and-Plate Control Upgrade:\n"
        "   Expanding the 1-DOF beam into a dual-axis servo gimbaling platform (X & Y axes)\n"
        "   utilizing overhead computer vision tracking for 2D trajectory tracing.\n\n"
        "2. Model Predictive Control (MPC) Implementation:\n"
        "   Deploying receding-horizon quadratic optimization with explicit state\n"
        "   and actuator constraints onto an embedded 32-bit ARM Cortex-M or ESP32.\n\n"
        "3. Drone Contactless Altitude Flight Controller:\n"
        "   Migrating our deterministic 5ms ultrasonic timeout and adaptive state-estimator\n"
        "   directly into autonomous quadcopter autopilots for rugged terrain-following."
    )
    ax.text(8.6, bottom_y + 0.25, future_text, color='#334155', fontsize=8.8, fontname=FONT_BODY, linespacing=1.32)

    fig.savefig(out_path, dpi=300, facecolor='#faf7f2')
    plt.close(fig)
    print(f"Generated Slide 11: {out_path}")


def generate_slide_12(out_path):
    """Slide 12: Real-World Applications (Heavy on applications!)"""
    fig = plt.figure(figsize=(16, 9), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor('#faf7f2')
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis('off')

    draw_canva_decorations(ax)

    # Page number box
    ax.add_patch(patches.Rectangle((1.0, 0.6), 0.75, 0.75, facecolor='#3f6588', edgecolor='none'))
    ax.text(1.375, 0.975, "12", color='#ffffff', fontsize=16, fontweight='bold', fontname=FONT_TITLE, ha='center', va='center')

    # Main Title
    ax.text(1.0, 8.0, "Real-World Industrial Relevance", 
            color='#1e3a5f', fontsize=28, fontweight='bold', fontname=FONT_TITLE)
    ax.text(1.0, 7.55, "Why This Plant Matters: From Undergraduate Benchmark to Aerospace & Robotics", 
            color='#5c768d', fontsize=12, fontweight='bold', fontname=FONT_BODY)

    # 4 Industrial Application Cards (2x2 Grid) with equal margins
    cards = [
        ("[1] ROCKET THRUST VECTOR CONTROL (TVC)",
         "SpaceX Falcon 9 / Starship / ISRO Launchers",
         "• Mathematical Equivalence: Tilting the rocket engine gimbal by angle θ to produce\n"
         "  lateral acceleration to keep an upright rocket balanced is mathematically\n"
         "  identical to tilting a beam to balance a rolling ball: G(s) ∝ 1/s².\n"
         "• Why Our Code Matters: Without derivative damping and anti-windup, the rocket\n"
         "  rapidly overcorrects and violently flips on vertical landing.",
         '#e0f2fe', '#0369a1'),

        ("[2] AUTONOMOUS DRONE ALTITUDE & TERRAIN FOLLOWING",
         "Delivery Quadcopters / Industrial Inspection UAVs",
         "• Ultrasonic Ground Ranging: Delivery drones use downward transducers to\n"
         "  maintain exact millimeter standoff heights above rolling hills and obstacles.\n"
         "• Safeguards in Action: Our 5ms echo timeout and outlier rejection filter ensure\n"
         "  that momentary acoustic scattering from grass or leaves never crashes the drone.",
         '#f0fdf4', '#15803d'),

        ("[3] SMART AGRICULTURE & GREENHOUSE GANTRY BOOMS",
         "Automated Spray Nozzles & Crop Lighting Regulators",
         "• Botanical Canopy Distance Keeping: Gantry booms must glide at an exact gap\n"
         "  (e.g. 20cm) above growing crops—too close crushes leaves, too far wastes spray.\n"
         "• Anticipatory Feedforward: Our velocity feedforward tilts the boom ahead of\n"
         "  measured motion, cutting tracking lag by 78% over uneven terrain.",
         '#fef3c7', '#b45309'),

        ("[4] LIQUID SLOSH SUPPRESSION IN TRANSPORT TANKERS",
         "Liquid Natural Gas (LNG), Chemicals & Fuel Trucks",
         "• Highway Dynamics: When heavy tankers brake or turn, fluid sloshes inside,\n"
         "  creating sudden center-of-gravity shifts that cause fatal truck rollovers.\n"
         "• Active Damping: Real-time stabilization rigs tilt counter to sloshing momentum,\n"
         "  using the exact same double-integrator damping algorithms developed here.",
         '#ede9fe', '#6d28d9')
    ]

    card_w = 6.7
    card_h = 2.6
    xs = [1.0, 8.3]
    ys = [4.6, 1.6]

    for idx, (title, sub, body, bg, border_col) in enumerate(cards):
        cx = xs[idx % 2]
        cy = ys[idx // 2]
        ax.add_patch(patches.FancyBboxPatch((cx, cy), card_w, card_h,
                                            boxstyle="round,pad=0.12,rounding_size=0.15",
                                            facecolor='#ffffff', edgecolor=border_col, linewidth=1.5))
        # Top banner in card
        ax.add_patch(patches.FancyBboxPatch((cx, cy + 1.95), card_w, 0.65,
                                            boxstyle="round,pad=0.08,rounding_size=0.1",
                                            facecolor=bg, edgecolor=border_col, linewidth=0.8))
        ax.text(cx + 0.2, cy + 2.35, title, color='#0f172a', fontsize=10.5, fontweight='bold', fontname=FONT_TITLE)
        ax.text(cx + 0.2, cy + 2.05, sub, color='#475569', fontsize=8.2, fontname=FONT_BODY, fontstyle='italic')
        ax.text(cx + 0.2, cy + 0.25, body, color='#334155', fontsize=8.2, fontname=FONT_BODY, linespacing=1.3)

    fig.savefig(out_path, dpi=300, facecolor='#faf7f2')
    plt.close(fig)
    print(f"Generated Slide 12: {out_path}")


def generate_slide_13(out_path):
    """Slide 13: Conclusion, Final Takeaways & Defense References with Cambria Typography"""
    fig = plt.figure(figsize=(16, 9), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor('#faf7f2')
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis('off')

    draw_canva_decorations(ax)

    # Page number box
    ax.add_patch(patches.Rectangle((1.0, 0.6), 0.75, 0.75, facecolor='#3f6588', edgecolor='none'))
    ax.text(1.375, 0.975, "13", color='#ffffff', fontsize=16, fontweight='bold', fontname=FONT_TITLE, ha='center', va='center')

    # Main Title & Subtitle
    ax.text(1.0, 8.0, "Conclusion & Project Defense Summary", 
            color='#1e3a5f', fontsize=28, fontweight='bold', fontname=FONT_TITLE)
    ax.text(1.0, 7.55, "Synthesis of Theoretical Formulations, Practical Safeguards & Research Validation", 
            color='#5c768d', fontsize=12, fontweight='bold', fontname=FONT_BODY)

    # =========================================================================
    # LEFT CARD: Core Engineering Achievements (X: 1.0 to 9.2, W: 8.2, H: 5.4)
    # =========================================================================
    ax.add_patch(patches.FancyBboxPatch((1.0, 1.8), 8.2, 5.4,
                                        boxstyle="round,pad=0.15,rounding_size=0.18",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(1.3, 6.85, "Summary of Core Engineering Contributions", 
            color='#1e3a5f', fontsize=14, fontweight='bold', fontname=FONT_TITLE)

    summary_text = (
        "1. Stabilization of an Inherently Unstable Plant:\n"
        "   Successfully stabilized a pure double integrator (1/s²) using discrete PID\n"
        "   with derivative-on-measurement (-dy/dt) to eliminate actuator kick.\n\n"
        "2. Closed-Loop Stability Proof & Safety Bounds:\n"
        "   Analytically derived the Routh-Hurwitz boundary (Ki < 3.77) and verified a robust\n"
        "   Phase Margin of 52.4° in the frequency domain (exceeding standard 45° margins).\n\n"
        "3. Calibrated Cyber-Physical Digital Twin:\n"
        "   Eliminated manual guesswork via non-linear parameter estimation (SciPy L-BFGS-B),\n"
        "   achieving a 30.6% reduction in sim-to-real error using logged telemetry.\n\n"
        "4. Automated Auto-Tuning & Dynamic Feedforward:\n"
        "   Formulated detuned Ziegler-Nichols tuning and an ITAE optimizer (ts = 1.45s)\n"
        "   coupled with velocity feedforward cutting dynamic tracking lag by 77.8%.\n\n"
        "5. Production-Ready Firmware & Web Deployment:\n"
        "   Implemented deterministic 33.3 Hz scheduling, anti-windup, and 5ms echo timeouts\n"
        "   with a driverless 5-tab browser bench operating over Web Serial."
    )
    ax.text(1.3, 2.05, summary_text, color='#334155', fontsize=8.8, fontname=FONT_BODY, linespacing=1.35)

    # =========================================================================
    # RIGHT TOP CARD: Academic References (X: 9.6 to 15.0, W: 5.4, H: 2.7)
    # =========================================================================
    ax.add_patch(patches.FancyBboxPatch((9.6, 4.5), 5.4, 2.7,
                                        boxstyle="round,pad=0.12,rounding_size=0.15",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(9.9, 6.85, "Academic & Technical References", 
            color='#1e3a5f', fontsize=12.5, fontweight='bold', fontname=FONT_TITLE)

    refs_text = (
        "• Ogata, Katsuhiko\n"
        "   Modern Control Engineering, 5th Ed., Pearson.\n\n"
        "• Åström, K. J. & Murray, R. M.\n"
        "   Feedback Systems: An Introduction for Scientists\n"
        "   and Engineers, Princeton University Press.\n\n"
        "• Åström, K. J. & Hägglund, T.\n"
        "   Advanced PID Control: Theory & Design, ISA.\n\n"
        "• Project Code Repository & Documentation:\n"
        "   github.com/SrishtiDangi/PID-Ultrasonic-Tracking-System"
    )
    ax.text(9.9, 4.75, refs_text, color='#475569', fontsize=8.2, fontname=FONT_BODY, linespacing=1.28)

    # =========================================================================
    # RIGHT BOTTOM CARD: Final Defense Takeaway Banner (X: 9.6 to 15.0, W: 5.4, H: 2.3)
    # =========================================================================
    ax.add_patch(patches.FancyBboxPatch((9.6, 1.8), 5.4, 2.3,
                                        boxstyle="round,pad=0.12,rounding_size=0.15",
                                        facecolor='#ecfdf5', edgecolor='#10b981', linewidth=1.5))
    ax.text(9.9, 3.75, "FINAL DEFENSE TAKEAWAY", 
            color='#065f46', fontsize=10.5, fontweight='bold', fontname=FONT_TITLE)
    
    quote_text = (
        "\"The ball-on-beam is not an academic toy—it is\n"
        "the universal cyber-physical benchmark for controlling\n"
        "accelerating, untethered dynamic systems across\n"
        "aerospace, robotics, and smart agriculture.\""
    )
    ax.text(9.9, 2.15, quote_text, color='#064e3b', fontsize=8.8, fontname=FONT_BODY, fontstyle='italic', linespacing=1.35)

    # Bottom Footer Strip
    ax.text(8.0, 1.0, "Thank you, Sir! Srishti Dangi • Stuti Shukla • Vanshika welcome your questions.", 
            color='#64748b', fontsize=10.5, fontweight='bold', fontname=FONT_TITLE, fontstyle='italic', ha='center')

    fig.savefig(out_path, dpi=300, facecolor='#faf7f2')
    plt.close(fig)
    print(f"Generated Slide 13: {out_path}")


def main():
    root = os.path.dirname(os.path.abspath(__file__))
    docs = os.path.join(root, "Docs")
    
    s11_path = os.path.join(root, "ppt_slide_11.png")
    s12_path = os.path.join(root, "ppt_slide_12.png")
    s13_path = os.path.join(root, "ppt_slide_13.png")

    generate_slide_11(s11_path)
    generate_slide_12(s12_path)
    generate_slide_13(s13_path)

    generate_slide_11(os.path.join(docs, "ppt_slide_11.png"))
    generate_slide_12(os.path.join(docs, "ppt_slide_12.png"))
    generate_slide_13(os.path.join(docs, "ppt_slide_13.png"))

if __name__ == "__main__":
    main()
