"""
generate_final_slides.py
========================
Generates full 16:9 presentation slide graphics for Slides 11, 12, and 13
matching the exact Canva template design (cream background, blue decorative
circles, rounded cards, and clean typography).
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def draw_canva_decorations(ax):
    """Draws subtle decorative circular accents matching the Canva template."""
    # Top-right corner circles
    ax.add_patch(patches.Circle((15.2, 8.2), 1.4, facecolor='#5c82a6', alpha=0.35, edgecolor='none'))
    ax.add_patch(patches.Circle((13.8, 8.8), 0.9, facecolor='#486b88', alpha=0.45, edgecolor='none'))
    ax.add_patch(patches.Circle((14.5, 6.8), 0.6, facecolor='none', edgecolor='#5c82a6', linewidth=2.0, alpha=0.6))
    ax.add_patch(patches.Circle((12.8, 8.0), 0.7, facecolor='none', edgecolor='#5c82a6', linewidth=1.5, alpha=0.5))

    # Bottom-right corner circles
    ax.add_patch(patches.Circle((15.0, 0.8), 1.6, facecolor='#5c82a6', alpha=0.45, edgecolor='none'))
    ax.add_patch(patches.Circle((13.4, 0.4), 0.8, facecolor='#486b88', alpha=0.3, edgecolor='none'))
    ax.add_patch(patches.Circle((14.0, 1.8), 0.7, facecolor='none', edgecolor='#5c82a6', linewidth=2.0, alpha=0.6))
    ax.add_patch(patches.Circle((12.5, 1.0), 0.9, facecolor='none', edgecolor='#5c82a6', linewidth=1.5, alpha=0.5))

    # Top-left corner subtle accent
    ax.add_patch(patches.Circle((0.8, 8.4), 1.0, facecolor='#5c82a6', alpha=0.25, edgecolor='none'))
    ax.add_patch(patches.Circle((1.8, 8.8), 0.5, facecolor='none', edgecolor='#5c82a6', linewidth=1.5, alpha=0.4))


def generate_slide_11(out_path):
    """Slide 11: Cyber-Physical Architecture & Key Results"""
    fig = plt.figure(figsize=(16, 9), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor('#faf7f2')
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis('off')

    draw_canva_decorations(ax)

    # Page number box bottom-left
    ax.add_patch(patches.Rectangle((0.8, 0.6), 0.8, 0.8, facecolor='#3f6588', edgecolor='none'))
    ax.text(1.2, 1.0, "11", color='#ffffff', fontsize=18, fontweight='bold', ha='center', va='center')

    # Main Title
    ax.text(1.2, 7.8, "Cyber-Physical Architecture & Results", 
            color='#294b6e', fontsize=30, fontweight='black', family='serif')

    # Top Pipeline Card (Spans across)
    ax.add_patch(patches.FancyBboxPatch((1.2, 5.4), 13.6, 1.8,
                                        boxstyle="round,pad=0.15,rounding_size=0.2",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(1.5, 6.85, "DATA PIPELINE: FROM TELEMETRY TO DIGITAL TWIN", 
            color='#1e3a5f', fontsize=12, fontweight='bold')

    # 4 Pipeline Stage Blocks
    stages = [
        ("1. Real CSV Logs", "logs/step_test.csv\n• 33.3 Hz sampling\n• Setpoint, echo, u(t)", '#e0f2fe', '#0369a1'),
        ("2. System ID Optimizer", "system_identification.py\n• SciPy L-BFGS-B\n• Non-linear curve fit", '#f0fdf4', '#15803d'),
        ("3. Calibrated Model", "identified_plant.json\n• Friction b=0.082 N·s/m\n• Servo lag τ=65 ms", '#fef3c7', '#b45309'),
        ("4. Interactive Twin", "index.html / simulator.py\n• 60 FPS live vectors\n• Bode stability audit", '#ede9fe', '#6d28d9')
    ]

    for i, (title, desc, bg, text_col) in enumerate(stages):
        bx = 1.5 + i * 3.3
        ax.add_patch(patches.FancyBboxPatch((bx, 5.6), 3.0, 1.05,
                                            boxstyle="round,pad=0.08,rounding_size=0.1",
                                            facecolor=bg, edgecolor='#cbd5e1', linewidth=1))
        ax.text(bx + 0.15, 6.35, title, color=text_col, fontsize=10.5, fontweight='bold')
        ax.text(bx + 0.15, 5.75, desc, color='#334155', fontsize=8.5, linespacing=1.2)
        if i < 3:
            ax.annotate('', xy=(bx + 3.25, 6.12), xytext=(bx + 3.05, 6.12),
                        arrowprops=dict(arrowstyle="->", color='#64748b', lw=2.5))

    # Left Bottom Card: Key Quantitative Results
    ax.add_patch(patches.FancyBboxPatch((1.2, 1.7), 6.6, 3.4,
                                        boxstyle="round,pad=0.15,rounding_size=0.2",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(1.5, 4.75, "Key Measured Results", color='#1e3a5f', fontsize=14, fontweight='bold')

    results_text = (
        "• Sim-to-Real Model Fit:\n"
        "   Model RMSE reduced from 2.14 cm → 1.48 cm (30.6% error reduction)\n\n"
        "• Identified Physical Parameters:\n"
        "   - Actuator Lag Time Constant: τ(servo) ≈ 65 ms\n"
        "   - Viscous Rolling Friction: b ≈ 0.082 N·s/m\n\n"
        "• Dynamic Settling Performance:\n"
        "   - Settling Time (ts 2%): 1.45 s (ITAE) / 1.67 s (Nominal PID)\n"
        "   - Peak Overshoot (Mp): 1.8% to 4.8% (zero destructive ringing)\n"
        "   - Steady-State Offset (ess): < 0.05 cm (zero offset under friction)"
    )
    ax.text(1.5, 2.0, results_text, color='#334155', fontsize=9.2, linespacing=1.35)

    # Right Bottom Card: Future Scope & Extensions
    ax.add_patch(patches.FancyBboxPatch((8.2, 1.7), 6.6, 3.4,
                                        boxstyle="round,pad=0.15,rounding_size=0.2",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(8.5, 4.75, "Future Scope & Platform Extensions", color='#1e3a5f', fontsize=14, fontweight='bold')

    future_text = (
        "1. 2-DOF Ball-and-Plate Upgrade:\n"
        "   Expanding from 1-DOF beam to dual-axis servo gimbaling (X & Y axes)\n"
        "   using overhead computer vision tracking.\n\n"
        "2. Model Predictive Control (MPC):\n"
        "   Implementing receding-horizon optimization with explicit constraint\n"
        "   handling running on embedded ARM Cortex-M or ESP32.\n\n"
        "3. Quadcopter Altitude Hold Integration:\n"
        "   Migrating the 5ms ultrasonic timeout and adaptive Kalman filter\n"
        "   into autonomous drone flight controllers for terrain following."
    )
    ax.text(8.5, 2.05, future_text, color='#334155', fontsize=9.2, linespacing=1.35)

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
    ax.add_patch(patches.Rectangle((0.8, 0.6), 0.8, 0.8, facecolor='#3f6588', edgecolor='none'))
    ax.text(1.2, 1.0, "12", color='#ffffff', fontsize=18, fontweight='bold', ha='center', va='center')

    # Main Title
    ax.text(1.2, 7.8, "Real-World Industrial Relevance", 
            color='#294b6e', fontsize=30, fontweight='black', family='serif')
    ax.text(1.2, 7.35, "Why This Plant Matters: From Undergraduate Benchmark to Aerospace & Robotics", 
            color='#5c768d', fontsize=12, fontweight='bold')

    # 4 Industrial Application Cards (2x2 Grid)
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

    coords = [(1.2, 4.45), (8.2, 4.45), (1.2, 1.6), (8.2, 1.6)]

    for idx, (title, sub, body, bg, border_col) in enumerate(cards):
        cx, cy = coords[idx]
        ax.add_patch(patches.FancyBboxPatch((cx, cy), 6.6, 2.65,
                                            boxstyle="round,pad=0.12,rounding_size=0.15",
                                            facecolor='#ffffff', edgecolor=border_col, linewidth=1.5))
        # Top banner in card
        ax.add_patch(patches.FancyBboxPatch((cx, cy + 1.95), 6.6, 0.70,
                                            boxstyle="round,pad=0.08,rounding_size=0.1",
                                            facecolor=bg, edgecolor=border_col, linewidth=0.8))
        ax.text(cx + 0.2, cy + 2.35, title, color='#0f172a', fontsize=11, fontweight='bold')
        ax.text(cx + 0.2, cy + 2.05, sub, color='#475569', fontsize=8.5, fontstyle='italic')

        ax.text(cx + 0.2, cy + 0.25, body, color='#334155', fontsize=8.2, linespacing=1.3)

    fig.savefig(out_path, dpi=300, facecolor='#faf7f2')
    plt.close(fig)
    print(f"Generated Slide 12: {out_path}")


def generate_slide_13(out_path):
    """Slide 13: Conclusion, Final Takeaways & Defense References"""
    fig = plt.figure(figsize=(16, 9), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor('#faf7f2')
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis('off')

    draw_canva_decorations(ax)

    # Page number box
    ax.add_patch(patches.Rectangle((0.8, 0.6), 0.8, 0.8, facecolor='#3f6588', edgecolor='none'))
    ax.text(1.2, 1.0, "13", color='#ffffff', fontsize=18, fontweight='bold', ha='center', va='center')

    # Main Title
    ax.text(1.2, 7.8, "Conclusion & Project Defense Summary", 
            color='#294b6e', fontsize=30, fontweight='black', family='serif')

    # Left Card: Core Engineering Achievements
    ax.add_patch(patches.FancyBboxPatch((1.2, 2.1), 8.0, 5.1,
                                        boxstyle="round,pad=0.15,rounding_size=0.2",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(1.5, 6.75, "Summary of Engineering Contributions", color='#1e3a5f', fontsize=15, fontweight='bold')

    summary_text = (
        "1. Solved an Inherently Unstable Plant:\n"
        "   Successfully stabilized a pure double integrator (1/s²) using discrete PID\n"
        "   with derivative-on-measurement (-dy/dt), eliminating derivative kick.\n\n"
        "2. Closed-Loop Stability Proof:\n"
        "   Analytically derived the Routh-Hurwitz boundary (Ki < 3.77) and verified an\n"
        "   exceptional Phase Margin of 52.4° in the frequency domain.\n\n"
        "3. Calibrated Cyber-Physical Digital Twin:\n"
        "   Replaced guesswork with non-linear system identification, cutting sim-to-real\n"
        "   discrepancy by 30.6% using real hardware CSV telemetry.\n\n"
        "4. Automated Auto-Tuning & Benchmarking:\n"
        "   Formulated detuned Ziegler-Nichols rules and an ITAE optimizer, slashing settling\n"
        "   time to 1.45s and eliminating 78% of dynamic tracking lag via feedforward.\n\n"
        "5. Accessible Browser-Based Deployment:\n"
        "   Engineered a 5-tab flagship web bench communicating driverless over USB Web Serial."
    )
    ax.text(1.5, 2.35, summary_text, color='#334155', fontsize=9.2, linespacing=1.35)

    # Right Top Card: Academic References
    ax.add_patch(patches.FancyBboxPatch((9.6, 4.45), 5.2, 2.75,
                                        boxstyle="round,pad=0.12,rounding_size=0.15",
                                        facecolor='#ffffff', edgecolor='#cbd5e1', linewidth=1.5))
    ax.text(9.9, 6.75, "Academic & Technical References", color='#1e3a5f', fontsize=13, fontweight='bold')

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
    ax.text(9.9, 4.65, refs_text, color='#475569', fontsize=8.2, linespacing=1.25)

    # Right Bottom Card: Final Defense Closing Thought
    ax.add_patch(patches.FancyBboxPatch((9.6, 2.1), 5.2, 2.1,
                                        boxstyle="round,pad=0.12,rounding_size=0.15",
                                        facecolor='#ecfdf5', edgecolor='#10b981', linewidth=1.5))
    ax.text(9.9, 3.8, "FINAL TAKEAWAY", color='#065f46', fontsize=11, fontweight='black')
    
    quote_text = (
        "\"The ball-on-beam is not a toy—it is the universal\n"
        "cyber-physical benchmark for controlling accelerating,\n"
        "untethered dynamic systems across aerospace,\n"
        "robotics, and autonomous vehicles.\""
    )
    ax.text(9.9, 2.4, quote_text, color='#064e3b', fontsize=9.2, fontweight='semibold', fontstyle='italic', linespacing=1.3)

    # Bottom Footer Strip
    ax.text(8.0, 1.0, "Thank you, Sir! Srishti Dangi • Stuti Shukla • Vanshika welcome your questions.", 
            color='#64748b', fontsize=11, fontweight='bold', ha='center')

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

    # Also backup to Docs/
    generate_slide_11(os.path.join(docs, "ppt_slide_11.png"))
    generate_slide_12(os.path.join(docs, "ppt_slide_12.png"))
    generate_slide_13(os.path.join(docs, "ppt_slide_13.png"))

if __name__ == "__main__":
    main()
