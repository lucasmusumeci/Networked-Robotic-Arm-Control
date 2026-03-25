"""
analyse_step.py  —  Extract identification parameters from a joint velocity step CSV.

The measured transfer function is:
    H(p) = qdot_simu / qdot_cmd = K*G * wn^2 / (p^2 + 2*zeta*wn*p + wn^2)

Because we identify velocity directly (not position), K*G is read straight
from the steady-state ratio qdot_simu_ss / qdot_cmd — no slope estimation needed.
Trc is the dead time before qdot_simu first begins to rise.

Usage:
    python3 analyse_step.py joint2_step.csv [--plot]

Output:
    Trc   : dead time (s)  — pure actuator lag, no UDP delay
    K*G   : static velocity gain (qdot_simu_ss / qdot_cmd)
    wn    : undamped natural frequency (rad/s)
    zeta  : damping ratio
    Kp_s  : suggested proportional gain for the correction loop
    Kd_s  : suggested derivative gain

Requires: numpy, scipy, matplotlib (optional, only for --plot)
"""

import sys
import csv
import numpy as np
from scipy.optimize import curve_fit

# ------------------------------------------------------------------ #
#  Load CSV                                                            #
# ------------------------------------------------------------------ #

def load_csv(path):
    t, qdot_cmd, qdot_simu = [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t_us"]) * 1e-6)
            qdot_cmd.append(float(row["qdot_cmd"]))
            qdot_simu.append(float(row["qdot_simu"]))
    return np.array(t), np.array(qdot_cmd), np.array(qdot_simu)

# ------------------------------------------------------------------ #
#  Dead time (Trc)                                                     #
# ------------------------------------------------------------------ #

def estimate_deadtime(t, qdot_simu, qdot_cmd, threshold_frac=0.05):
    """
    First time qdot_simu exceeds threshold_frac of the commanded step.
    With velocity measurement this is easy to read: the signal is flat
    at zero, then rises — no integrator drift to contend with.
    """
    V = np.max(np.abs(qdot_cmd))
    thresh = threshold_frac * V

    for i in range(1, len(qdot_simu)):
        if abs(qdot_simu[i]) > thresh:
            frac = (thresh - abs(qdot_simu[i-1])) / \
                   max(abs(qdot_simu[i] - qdot_simu[i-1]), 1e-12)
            return t[i-1] + frac * (t[i] - t[i-1])
    return t[-1]

# ------------------------------------------------------------------ #
#  Static gain K*G                                                     #
# ------------------------------------------------------------------ #

def estimate_gain(t, qdot_cmd, qdot_simu, trc, T_excite):
    """
    K*G = mean(qdot_simu) / qdot_cmd  during the steady-state window.
    With velocity identification this is direct — no slope needed.
    Window: [trc + 0.4*T_excite , 0.85*T_excite]
    """
    t_start = trc + 0.4 * T_excite
    t_end   = 0.85 * T_excite
    mask    = (t >= t_start) & (t <= t_end)

    if mask.sum() < 4:
        print("  WARNING: too few steady-state points for gain estimate.")
        return float("nan")

    V_cmd = np.mean(qdot_cmd[mask])
    if abs(V_cmd) < 1e-9:
        return float("nan")
    return np.mean(qdot_simu[mask]) / V_cmd

# ------------------------------------------------------------------ #
#  2nd-order fit on the velocity step response                         #
# ------------------------------------------------------------------ #

def second_order_velocity_step(t, wn, zeta, KG):
    """
    Unit-step response of  H(s) = KG*wn^2 / (s^2 + 2*zeta*wn*s + wn^2).
    This is directly the shape of qdot_simu after dead-time removal.
    """
    if zeta >= 1.0:
        # Over-damped: two real poles
        r = np.sqrt(zeta**2 - 1.0)
        s1, s2 = -wn * (zeta - r), -wn * (zeta + r)
        return KG * (1.0 + (s2 * np.exp(s1 * t) - s1 * np.exp(s2 * t)) / (s1 - s2))
    # Under-damped
    wd = wn * np.sqrt(1.0 - zeta**2)
    return KG * (1.0 - np.exp(-zeta * wn * t) *
                 (np.cos(wd * t) + (zeta / np.sqrt(1.0 - zeta**2)) * np.sin(wd * t)))

def estimate_second_order(t, qdot_cmd, qdot_simu, trc, KG, T_excite):
    """
    Shift time to remove dead time, normalise to a unit step, and fit wn / zeta.
    """
    mask = (t >= trc) & (t <= 0.9 * T_excite)
    if mask.sum() < 6:
        return float("nan"), float("nan")

    t_fit   = t[mask] - trc
    V       = np.mean(qdot_cmd[mask])
    # Normalise: 0 → 0, 1 → steady state (= K*G for a unit step)
    y_fit   = qdot_simu[mask] / (V if abs(V) > 1e-9 else 1.0)

    try:
        kg0    = KG if not np.isnan(KG) else 1.0
        p0     = [6.0, 0.6, kg0]
        bounds = ([0.1, 0.01, 0.01], [200.0, 2.0, 20.0])
        popt, _ = curve_fit(second_order_velocity_step, t_fit, y_fit,
                            p0=p0, bounds=bounds, maxfev=8000)
        wn, zeta, _ = popt
        return wn, zeta
    except RuntimeError:
        print("  WARNING: 2nd-order fit did not converge.")
        return float("nan"), float("nan")

# ------------------------------------------------------------------ #
#  Suggested PD gains                                                  #
# ------------------------------------------------------------------ #

def suggest_gains(wn, zeta, trc, safety_factor=0.35):
    """
    Phase-margin budget of 45° with delay Trc:
        wc_max = (pi/2 - 10°) / Trc
        wc     = safety_factor * wc_max
    PD gains for the identified 2nd-order velocity plant:
        Kp = (wc/wn)^2
        Kd = 2*zeta*wc / wn^2
    """
    if any(np.isnan(x) for x in [wn, zeta]) or trc < 1e-6:
        return float("nan"), float("nan")
    pm_budget = np.pi / 2.0 - np.radians(10.0)
    wc_max    = pm_budget / trc
    wc        = safety_factor * wc_max
    Kp_s      = (wc / wn) ** 2
    Kd_s      = 2.0 * zeta * wc / wn**2
    return Kp_s, Kd_s

# ------------------------------------------------------------------ #
#  Main                                                                #
# ------------------------------------------------------------------ #

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    csv_path = sys.argv[1]
    do_plot  = "--plot" in sys.argv

    t, qdot_cmd, qdot_simu = load_csv(csv_path)

    # Detect step timing from the command signal
    step_on = qdot_cmd != 0.0
    if not step_on.any():
        print("ERROR: no non-zero qdot_cmd found in the file.")
        sys.exit(1)

    T_step_start = t[np.argmax(step_on)]
    last_on      = len(step_on) - 1 - np.argmax(step_on[::-1])
    T_excite     = t[last_on] - T_step_start
    V_step       = float(qdot_cmd[step_on][0])

    print(f"\nFile       : {csv_path}")
    print(f"Step amp   : {V_step:.4f} rad/s")
    print(f"T_excite   : {T_excite:.3f} s")

    trc        = estimate_deadtime(t, qdot_simu, qdot_cmd)
    KG         = estimate_gain(t, qdot_cmd, qdot_simu, trc, T_excite)
    wn, zeta   = estimate_second_order(t, qdot_cmd, qdot_simu, trc, KG, T_excite)
    Kp_s, Kd_s = suggest_gains(wn, zeta, trc)

    print(f"\n--- Identification results ---")
    print(f"  Trc   = {trc:.4f} s   (pure actuator dead time, no UDP delay)")
    print(f"  K*G   = {KG:.4f}     (static velocity gain)")
    print(f"  wn    = {wn:.4f} rad/s")
    print(f"  zeta  = {zeta:.4f}")
    print(f"\n--- Suggested PD gains (safety factor {0.35}) ---")
    print(f"  Kp    = {Kp_s:.5f}")
    print(f"  Kd    = {Kd_s:.5f}")
    print(f"  (Add 0.5 s network delay on top of Trc for the Smith Predictor)")

    if do_plot:
        try:
            import matplotlib.pyplot as plt

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

            ax1.step(t, qdot_cmd, "k--", where="post", label="qdot_cmd", lw=1)
            ax1.set_ylabel("Velocity command (rad/s)")
            ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)

            ax2.plot(t, qdot_simu, "b", label="qdot_simu", lw=1.2)
            ax2.axvline(trc, color="r", linestyle=":", lw=1,
                        label=f"Trc = {trc:.3f} s")

            # Overlay 2nd-order model
            if not (np.isnan(wn) or np.isnan(zeta)):
                mask_fit = t >= trc
                t_fit    = t[mask_fit] - trc
                y_model  = V_step * second_order_velocity_step(t_fit, wn, zeta, KG)
                ax2.plot(t[mask_fit], y_model, "r--", lw=1,
                         label=f"2nd-order fit  wn={wn:.2f} ζ={zeta:.2f} K*G={KG:.3f}")

            ax2.set_ylabel("Joint velocity (rad/s)")
            ax2.set_xlabel("Time (s)")
            ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

            plt.suptitle(csv_path, fontsize=10)
            plt.tight_layout()
            plt.show()

        except ImportError:
            print("matplotlib not installed — skipping plot.")