"""
analyse_step.py  —  First-order identification from a joint velocity step CSV.

Model identified:
    H(p) = qdot_simu / qdot_cmd = K*G * e^(-Trc*p) / (tau*p + 1)

Rationale: commanding velocity and measuring velocity removes the mechanical
integrator 1/p from the observed signal. What remains is a first-order lag
(time constant tau) plus a pure dead time Trc — no second-order dynamics.

Parameters extracted:
    Trc   : dead time (s)     — delay before qdot_simu begins to move
    K*G   : static gain       — qdot_simu_ss / qdot_cmd (should be ≈ 1)
    tau   : time constant (s) — 63% rise time after dead time removal

Usage:
    python3 analyse_step.py joint2_step.csv [--plot]

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
            t.append(float(row["t_ms"]) * 1e-3)
            qdot_cmd.append(float(row["qdot_cmd"]))
            qdot_simu.append(float(row["qdot_simu"]))
    return np.array(t), np.array(qdot_cmd), np.array(qdot_simu)

# ------------------------------------------------------------------ #
#  Dead time Trc                                                       #
# ------------------------------------------------------------------ #

def estimate_deadtime(t, qdot_simu, qdot_cmd, threshold_frac=0.05):
    """
    First sample where |qdot_simu| exceeds threshold_frac of the step amplitude.
    Interpolated for sub-sample precision.
    """
    V      = np.max(np.abs(qdot_cmd))
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
    Mean of qdot_simu in the steady-state window divided by qdot_cmd.
    Window chosen well after the transient has settled: [trc + 0.5*T_excite, 0.9*T_excite].
    """
    t_start = trc + 0.5 * T_excite
    t_end   = 0.90 * T_excite
    mask    = (t >= t_start) & (t <= t_end)
    if mask.sum() < 4:
        print("  WARNING: too few steady-state points for gain estimate.")
        return float("nan")
    V_cmd = np.mean(qdot_cmd[mask])
    if abs(V_cmd) < 1e-9:
        return float("nan")
    return np.mean(qdot_simu[mask]) / V_cmd

# ------------------------------------------------------------------ #
#  First-order fit                                                     #
# ------------------------------------------------------------------ #

def first_order_step(t, tau, KG):
    """
    Step response of  H(p) = KG / (tau*p + 1)  — i.e. KG*(1 - exp(-t/tau)).
    t is already shifted so that t=0 is the moment the step arrives (Trc removed).
    """
    return KG * (1.0 - np.exp(-t / tau))

def estimate_first_order(t, qdot_cmd, qdot_simu, trc, KG, T_excite):
    """
    Shift time axis to remove dead time, then fit tau (and refine KG).
    Only data in the excitation phase is used for the fit.
    """
    mask = (t >= trc) & (t <= 0.95 * T_excite)
    if mask.sum() < 6:
        print("  WARNING: too few points in excitation phase for tau fit.")
        return float("nan"), KG

    t_fit = t[mask] - trc
    V     = np.mean(qdot_cmd[mask])
    y_fit = qdot_simu[mask] / (V if abs(V) > 1e-9 else 1.0)   # normalise to unit step

    kg0 = KG if not np.isnan(KG) else 1.0
    # Ensure upper bound for tau is larger than lower bound
    tau_upper = max(T_excite, 1.0)
    try:
        popt, pcov = curve_fit(
            first_order_step, t_fit, y_fit,
            p0=[0.2, kg0],
            bounds=([1e-4, 0.01], [tau_upper, 20.0]),
            maxfev=8000
        )
        tau_fit, kg_fit = popt
        return tau_fit, kg_fit
    except RuntimeError:
        print("  WARNING: first-order curve fit did not converge.")
        return float("nan"), KG

# ------------------------------------------------------------------ #
#  Suggested proportional gain for a P controller with delay           #
# ------------------------------------------------------------------ #

def suggest_gain_P(KG, tau, trc):
    """
    For H(p) = KG/(tau*p+1) with delay Trc, a pure P controller C(p) = K gives:
        L(p) = K*KG*e^{-Trc*p} / (tau*p+1)

    Phase margin condition (crossover at wc):
        PM = pi - pi/2 - wc*Trc - arctan(wc*tau)  =  45°  (target)

    A practical conservative rule (AMIGO-like, valid when Trc/tau is small):
        K_suggested = tau / (KG * (Trc + tau/2))

    This keeps PM ≈ 60° for small Trc/tau ratios.
    """
    if any(np.isnan(x) for x in [KG, tau]) or trc < 1e-6 or KG < 1e-9:
        return float("nan")
    return tau / (KG * (trc + tau / 2.0))

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

    # Detect step timing
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

    trc          = estimate_deadtime(t, qdot_simu, qdot_cmd)
    KG           = estimate_gain(t, qdot_cmd, qdot_simu, trc, T_excite)
    tau, KG_fit  = estimate_first_order(t, qdot_cmd, qdot_simu, trc, KG, T_excite)
    K_suggested  = suggest_gain_P(KG_fit, tau, trc)

    print(f"\n--- First-order identification ---")
    print(f"  Trc   = {trc:.4f} s      (actuator dead time, no UDP delay)")
    print(f"  K*G   = {KG_fit:.4f}        (static velocity gain, ideally ≈ 1)")
    print(f"  tau   = {tau:.4f} s      (time constant)")
    print(f"\n  Model: H(p) = {KG_fit:.3f} * exp(-{trc:.3f}*p) / ({tau:.3f}*p + 1)")
    print(f"\n--- Suggested P gain (AMIGO rule, PM ≈ 60°) ---")
    print(f"  K     = {K_suggested:.5f}")
    print(f"\n  NOTE: add 0.5 s UDP delay on top of Trc when computing Trc_max")
    print(f"        for the stability widget: Trc_total = {trc + 0.5:.4f} s")

    if do_plot:
        try:
            import matplotlib.pyplot as plt

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

            ax1.step(t, qdot_cmd, "k--", where="post", label="qdot_cmd", lw=1)
            ax1.set_ylabel("Velocity command (rad/s)")
            ax1.legend(fontsize=8)
            ax1.grid(True, alpha=0.3)

            ax2.plot(t, qdot_simu, "b", label="qdot_simu (measured)", lw=1.2)
            ax2.axvline(trc, color="r", linestyle=":", lw=1,
                        label=f"Trc = {trc:.3f} s")

            # Overlay first-order model
            if not np.isnan(tau):
                mask_fit = t >= trc
                t_fit    = t[mask_fit] - trc
                y_model  = V_step * first_order_step(t_fit, tau, KG_fit)
                ax2.plot(t[mask_fit], y_model, "r--", lw=1.5,
                         label=f"1st-order fit:  K·G={KG_fit:.3f}  τ={tau:.3f} s")

                # Mark the 63% point (one time constant)
                y63 = V_step * KG_fit * 0.632
                ax2.axhline(y63, color="orange", linestyle=":", lw=0.8,
                            label=f"63% = {y63:.3f} rad/s  (at t = Trc + τ)")
                ax2.axvline(trc + tau, color="orange", linestyle=":", lw=0.8)

            ax2.set_ylabel("Joint velocity (rad/s)")
            ax2.set_xlabel("Time (s)")
            ax2.legend(fontsize=8)
            ax2.grid(True, alpha=0.3)

            plt.suptitle(f"{csv_path}  —  1st-order fit", fontsize=10)
            plt.tight_layout()
            plt.show()

        except ImportError:
            print("matplotlib not installed — skipping plot.")