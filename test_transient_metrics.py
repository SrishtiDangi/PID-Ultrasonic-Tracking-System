"""
================================================================================
UNIT TEST: Transient Metrics Verification Against Analytic 2nd-Order Prototype
Validates Rise Time, Settling Time (2%), and Peak Overshoot against theoretical formulas
================================================================================
"""

import unittest
import numpy as np


def compute_transient_metrics(t, y, r_initial, r_final):
    """
    Computes rise time (10-90%), settling time (2% band), peak overshoot %,
    steady-state error, ITAE, and RMSE from a time-series response.
    """
    step_mag = abs(r_final - r_initial)
    if step_mag < 1e-4:
        return {}

    direction = 1.0 if r_final >= r_initial else -1.0

    # 1. Peak Overshoot
    if direction > 0:
        peak_val = np.max(y)
        peak_idx = np.argmax(y)
        overshoot = max(0.0, (peak_val - r_final) / step_mag * 100.0)
    else:
        peak_val = np.min(y)
        peak_idx = np.argmin(y)
        overshoot = max(0.0, (r_final - peak_val) / step_mag * 100.0)

    # 2. Rise Time (10% to 90%)
    val_10 = r_initial + 0.10 * (r_final - r_initial)
    val_90 = r_initial + 0.90 * (r_final - r_initial)

    idx_10 = np.where((y >= val_10) if direction > 0 else (y <= val_10))[0]
    idx_90 = np.where((y >= val_90) if direction > 0 else (y <= val_90))[0]

    t_rise = None
    if len(idx_10) > 0 and len(idx_90) > 0:
        t_10 = t[idx_10[0]]
        t_90 = t[idx_90[0]]
        if t_90 >= t_10:
            t_rise = float(t_90 - t_10)

    # 3. Settling Time (2% error envelope)
    band = 0.02 * step_mag
    within_band = np.abs(y - r_final) <= band
    t_settling = None
    for i in range(len(y) - 1, -1, -1):
        if not within_band[i]:
            if i + 1 < len(t):
                t_settling = float(t[i + 1] - t[0])
            else:
                t_settling = float(t[-1] - t[0])
            break
    if t_settling is None and np.all(within_band):
        t_settling = float(t[0] - t[0])

    # 4. Steady-state error
    tail_len = max(1, int(len(y) * 0.10))
    y_ss = np.mean(y[-tail_len:])
    ess = float(abs(r_final - y_ss))

    # 5. ITAE and RMSE
    dt = np.mean(np.diff(t))
    err = np.abs(r_final - y)
    t_rel = t - t[0]
    itae = float(np.sum(t_rel * err * dt))
    rmse = float(np.sqrt(np.mean((r_final - y) ** 2)))

    return {
        "rise_time": t_rise,
        "settling_time": t_settling,
        "overshoot_pct": float(overshoot),
        "steady_state_error": ess,
        "itae": itae,
        "rmse": rmse
    }


class TestTransientMetrics(unittest.TestCase):
    def test_underdamped_analytic_second_order(self):
        """
        Analytic Prototype:
        G(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
        Choose zeta = 0.5, omega_n = 2.0 rad/s
        Analytic Step Response:
        y(t) = 1 - (e^(-zeta*omega_n*t) / sqrt(1-zeta^2)) * sin(omega_d*t + phi)
        """
        zeta = 0.50
        omega_n = 2.0
        omega_d = omega_n * np.sqrt(1.0 - zeta**2) # 2.0 * sqrt(0.75) ~ 1.732 rad/s
        phi = np.arccos(zeta) # arccos(0.5) = pi/3 rad

        # Theoretical values
        # Peak Overshoot: M_p = 100 * exp(-pi*zeta / sqrt(1-zeta^2))
        expected_Mp = 100.0 * np.exp(-np.pi * zeta / np.sqrt(1.0 - zeta**2)) # ~ 16.303%
        # Settling time (2%): approx 4 / (zeta * omega_n) = 4 / (0.5 * 2.0) = 4.00 s
        expected_ts = 4.0 / (zeta * omega_n)

        # Generate dense analytic time response
        dt = 0.005 # high resolution
        t = np.arange(0.0, 10.0, dt)
        decay = np.exp(-zeta * omega_n * t)
        y = 1.0 - (decay / np.sqrt(1.0 - zeta**2)) * np.sin(omega_d * t + phi)

        # Rise time (10% to 90%):
        t_10_analytic = t[np.where(y >= 0.10)[0][0]]
        t_90_analytic = t[np.where(y >= 0.90)[0][0]]
        expected_tr_10_90 = t_90_analytic - t_10_analytic

        metrics = compute_transient_metrics(t, y, r_initial=0.0, r_final=1.0)

        print("\n" + "=" * 60)
        print("ANALYTIC SECOND-ORDER STEP METRIC VERIFICATION:")
        print(f"  • Peak Overshoot : Expected {expected_Mp:.2f}% | Computed {metrics['overshoot_pct']:.2f}%")
        print(f"  • Rise Time (10-90%): Expected {expected_tr_10_90:.3f} s | Computed {metrics['rise_time']:.3f} s")
        print(f"  • Settling (2%)  : Expected ~{expected_ts:.2f} s | Computed {metrics['settling_time']:.2f} s")
        print(f"  • Steady Error   : Computed {metrics['steady_state_error']:.5f} cm")
        print("=" * 60)

        # Validate within tight engineering tolerances
        self.assertAlmostEqual(metrics['overshoot_pct'], expected_Mp, delta=0.5)
        self.assertAlmostEqual(metrics['rise_time'], expected_tr_10_90, delta=0.01)
        self.assertAlmostEqual(metrics['settling_time'], expected_ts, delta=0.35)
        self.assertLess(metrics['steady_state_error'], 0.005)


if __name__ == "__main__":
    unittest.main()
