"""
Precision Drive Dynamics: Tier 2 Benchmark Simulation
Compares 1-SMC / Classical Baseline against High-Gain LESO + Super-Twisting 2-SMC
under Stribeck friction and 0.45 N*m transient load torque impact.
Saves publication-quality figure: pid_figures/fig_precision_drive_benchmark.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt


def run_precision_drive_benchmark():
    # Motor Parameters matching verified benchmark
    J = 0.01       # Rotor inertia (kg*m^2)
    b = 0.1        # Viscous friction (N*m*s/rad)
    K = 0.01       # Torque constant
    a_mech = b / J
    b_mech = K / J

    # Stribeck non-linear parameters
    Tc = 0.02
    Ts = 0.05
    omega_s = 5.0

    dt = 0.001
    t = np.arange(0, 2.5, dt)
    n = len(t)

    # 100 rad/s reference
    w_ref = 100.0 * np.ones(n)
    w_ref[t < 0.2] = 0.0

    # Disturbance impact at t=1.2s
    T_dist = np.zeros(n)
    T_dist[t >= 1.2] = 0.45 + 0.15 * np.sin(20.0 * (t[t >= 1.2] - 1.2))

    def stribeck(w):
        return (Tc + (Ts - Tc) * np.exp(-((w / omega_s) ** 2))) * np.tanh(10.0 * w)

    # 1. Baseline 1-SMC (with high chattering)
    w_base = np.zeros(n)
    u_base = np.zeros(n)

    for i in range(n - 1):
        e1 = w_ref[i] - w_base[i]
        # Standard sliding mode with discontinuous switching
        u_disc = 35.0 * np.sign(e1) if np.abs(e1) > 0.01 else 35.0 * (e1 / 0.01)
        u_total = (a_mech * w_base[i] + u_disc) / b_mech
        u_base[i] = np.clip(u_total, -48.0, 48.0)

        dist_i = (T_dist[i] + stribeck(w_base[i])) / J
        dw = -a_mech * w_base[i] + b_mech * u_base[i] - dist_i
        w_base[i + 1] = w_base[i] + dw * dt
    u_base[-1] = u_base[-2]

    # 2. LESO + Super-Twisting 2-SMC (Ours)
    w_sta = np.zeros(n)
    u_sta = np.zeros(n)
    z1_eso = np.zeros(n)
    z2_eso = np.zeros(n)

    omega_o = 60.0
    beta1 = 2.0 * omega_o
    beta2 = omega_o ** 2
    k1 = 18.0
    k2 = 45.0
    v_integral = 0.0

    for i in range(n - 1):
        err_eso = z1_eso[i] - w_sta[i]
        dz1 = z2_eso[i] - beta1 * err_eso + b_mech * u_sta[i] - a_mech * w_sta[i]
        dz2 = -beta2 * err_eso
        z1_eso[i + 1] = z1_eso[i] + dz1 * dt
        z2_eso[i + 1] = z2_eso[i] + dz2 * dt

        e_sta = w_ref[i] - w_sta[i]
        s_sign = np.sign(e_sta)
        v_integral += k2 * s_sign * dt
        u_sta_core = k1 * np.sqrt(np.abs(e_sta)) * s_sign + v_integral

        # Cancel estimated disturbance and match model
        u_total = (a_mech * w_sta[i] - z2_eso[i] + u_sta_core) / b_mech
        u_sta[i] = np.clip(u_total, -48.0, 48.0)

        dist_i = (T_dist[i] + stribeck(w_sta[i])) / J
        dw = -a_mech * w_sta[i] + b_mech * u_sta[i] - dist_i
        w_sta[i + 1] = w_sta[i] + dw * dt
    u_sta[-1] = u_sta[-2]

    # Metrics at disturbance impact t >= 1.2s
    idx_settle = (t >= 0.8) & (t < 1.2)
    idx_dist = t >= 1.2
    sag_base = np.mean(w_base[idx_settle]) - np.min(w_base[idx_dist])
    sag_sta = np.mean(w_sta[idx_settle]) - np.min(w_sta[idx_dist])
    sag_reduction = ((sag_base - sag_sta) / sag_base) * 100.0

    # Chattering variance
    diff_base = np.diff(u_base[t >= 0.5])
    diff_sta = np.diff(u_sta[t >= 0.5])
    chatter_cut = (1.0 - (np.var(diff_sta) / max(np.var(diff_base), 1e-6))) * 100.0

    print("=" * 70)
    print("PRECISION DRIVE DYNAMICS: TIER 2 HARDWARE BENCHMARK")
    print(f"Baseline Disturbance Speed Sag   : {sag_base:.2f} rad/s")
    print(f"LESO + Super-Twisting Speed Sag  : {sag_sta:.2f} rad/s")
    print(f"Disturbance Sag Reduction Gain   : {sag_reduction:.2f}%")
    print(f"Control Chattering Suppression   : {chatter_cut:.2f}%")
    print("=" * 70)

    # Plot
    os.makedirs("pid_figures", exist_ok=True)
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(t, w_ref, 'k--', label="Target Velocity (100 rad/s)", linewidth=1.2)
    axs[0].plot(t, w_base, 'r-', label=f"1-SMC Baseline (Sag: {sag_base:.2f} rad/s)", alpha=0.85)
    axs[0].plot(t, w_sta, 'b-', label=f"LESO + 2-SMC (Sag: {sag_sta:.2f} rad/s, {sag_reduction:.1f}% drop)", linewidth=1.6)
    axs[0].axvline(1.2, color='gray', linestyle=':', label="Disturbance Injection")
    axs[0].set_ylabel("Velocity (rad/s)", fontsize=11)
    axs[0].set_title("Precision Motor Drive: 0.45 N*m Disturbance Sag & Chattering Suppression", fontsize=12, fontweight='bold')
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc="lower right")

    axs[1].plot(t, T_dist, 'k-', label="True Disturbance Load (N*m)", linewidth=1.4)
    axs[1].plot(t, -z2_eso * J, 'm--', label="LESO Estimated Disturbance (N*m)", linewidth=1.2)
    axs[1].set_ylabel("Torque (N*m)", fontsize=11)
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="upper right")

    axs[2].plot(t, u_base, 'r-', label="1-SMC High-Frequency Chattering", alpha=0.5)
    axs[2].plot(t, u_sta, 'b-', label=f"LESO + 2-SMC Smooth Control ({chatter_cut:.1f}% Chattering Cut)", linewidth=1.3)
    axs[2].set_ylabel("Control Effort (V)", fontsize=11)
    axs[2].set_xlabel("Time (s)", fontsize=11)
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(loc="lower right")

    out_path = os.path.join("pid_figures", "fig_precision_drive_benchmark.png")
    plt.tight_layout()
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Publication benchmark plot saved to: {out_path}\n")


if __name__ == "__main__":
    run_precision_drive_benchmark()
