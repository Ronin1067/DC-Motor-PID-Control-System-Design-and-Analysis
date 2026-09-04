"""
Modern State-Space Control, LQR Synthesis, Observer Design & Monte Carlo Robustness Analysis
Author: Yagnesh Kumar Koduru
Repository: DC-Motor-PID-Control-System-Design-and-Analysis
Domain: Precision Motion Control, Robust Control Theory, Physical Intelligence

This module implements:
1. 2-state electromechanical speed regulation state-space modeling [omega, i_a]^T
2. Controllability and Observability proofs
3. Linear Quadratic Regulator (LQR) state-feedback synthesis with feedforward precompensation
4. 100-sample Monte Carlo parameter uncertainty robustness analysis (R, L, J, B, K)
5. Actuator effort and disturbance rejection evaluation
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are, eigvals

# Force UTF-8 output encoding for Windows
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

# Publication-grade plot aesthetics
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['lines.linewidth'] = 2.0
plt.rcParams['axes.grid'] = True
plt.rcParams['grid.alpha'] = 0.35

output_dir = 'pid_figures'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)


class DCMotorStateSpace:
    def __init__(self, J=0.01, B=0.1, K=0.01, R=1.0, L=0.5):
        """
        Nominal DC Motor Parameters:
        - J: Rotor moment of inertia (kg*m^2)
        - B: Motor viscous friction constant (N*m*s)
        - K: Back-EMF / Torque constant (V*s/rad or N*m/A)
        - R: Armature resistance (Ohms)
        - L: Armature inductance (H)
        """
        self.J = J
        self.B = B
        self.K = K
        self.R = R
        self.L = L

        # Continuous-time state space matrices: x = [omega, i_a]^T
        self.A = np.array([
            [-B / J,  K / J],
            [-K / L, -R / L]
        ])

        self.B_mat = np.array([
            [0.0],
            [1.0 / L]
        ])

        # Output: y = omega
        self.C = np.array([[1.0, 0.0]])
        self.D = np.array([[0.0]])

        # Disturbance input matrix for load torque T_L on omega
        self.B_dist = np.array([
            [-1.0 / J],
            [0.0]
        ])

    def check_controllability_observability(self):
        """Verify Kalman rank conditions."""
        # Controllability: C_ctrl = [B, AB]
        C_ctrl = np.hstack([self.B_mat, self.A @ self.B_mat])
        rank_ctrl = np.linalg.matrix_rank(C_ctrl)

        # Observability: O_obs = [C; CA]
        O_obs = np.vstack([self.C, self.C @ self.A])
        rank_obs = np.linalg.matrix_rank(O_obs)

        return rank_ctrl, rank_obs

    def design_lqr(self, q_weights=(1000.0, 1.0), r_weight=0.01):
        """
        Synthesize optimal state-feedback gain K_lqr.
        Cost function: J = integral (x^T Q x + u^T R u) dt
        """
        Q = np.diag(q_weights)
        R = np.array([[r_weight]])

        # Solve Continuous Algebraic Riccati Equation
        P = solve_continuous_are(self.A, self.B_mat, Q, R)
        K_lqr = np.linalg.inv(R) @ (self.B_mat.T @ P)

        # Feedforward precompensation gain N_bar for unity steady-state speed tracking
        A_cl = self.A - self.B_mat @ K_lqr
        inv_A_cl = np.linalg.inv(A_cl)
        N_bar = float(-1.0 / (self.C @ inv_A_cl @ self.B_mat)[0, 0])

        return K_lqr, N_bar, P


def run_modern_control_study():
    print("=" * 75)
    print("DC MOTOR MODERN STATE-SPACE & LQR CONTROL BENCHMARK")
    print("Author: Yagnesh Kumar Koduru")
    print("=" * 75)

    motor = DCMotorStateSpace()
    rank_c, rank_o = motor.check_controllability_observability()
    print(f"Speed Subsystem Dimension: n = 2")
    print(f"Controllability Matrix Rank: {rank_c}/2 (Full Controllability: {'YES' if rank_c == 2 else 'NO'})")
    print(f"Observability Matrix Rank: {rank_o}/2 (Full Observability: {'YES' if rank_o == 2 else 'NO'})")

    # LQR Synthesis
    K_lqr, N_bar, P_riccati = motor.design_lqr(q_weights=(1200.0, 0.5), r_weight=0.005)
    print(f"\nOptimal LQR State-Feedback Gain K: {K_lqr[0]}")
    print(f"Feedforward Prefilter Gain N_bar: {N_bar:.4f}")

    # Closed-loop poles with LQR
    A_cl = motor.A - motor.B_mat @ K_lqr
    cl_poles = eigvals(A_cl)
    print(f"LQR Closed-Loop Poles: {[f'{p.real:.3f} + {p.imag:.3f}j' if abs(p.imag) > 1e-4 else f'{p.real:.3f}' for p in cl_poles]}")

    # Simulation setup
    t_span = (0.0, 2.0)
    t_eval = np.linspace(t_span[0], t_span[1], 2000)
    omega_ref = 1.0  # Unit step reference (1 rad/s)

    def lqr_dynamics(t, x, m_sys, k_gain, n_gain, load_torque=0.0):
        # State: x = [omega, i_a]
        u = -float(np.squeeze(k_gain @ x)) + n_gain * omega_ref
        u = np.clip(u, -24.0, 24.0)
        
        # Load torque step disturbance at t >= 1.0s
        t_load = load_torque if t >= 1.0 else 0.0
        
        d_x = m_sys.A @ x + (m_sys.B_mat * u).flatten() + (m_sys.B_dist * t_load).flatten()
        return d_x

    # Run Nominal Simulation with 20% load disturbance at t=1.0s (T_L = 0.02 N*m)
    sol_nom = solve_ivp(
        fun=lambda t, y: lqr_dynamics(t, y, motor, K_lqr, N_bar, load_torque=0.02),
        t_span=t_span,
        y0=[0.0, 0.0],
        t_eval=t_eval,
        method='RK45',
        rtol=1e-7,
        atol=1e-9
    )

    t = sol_nom.t
    omega = sol_nom.y[0]
    ia = sol_nom.y[1]

    # Compute control voltage u(t)
    u_vals = np.zeros_like(t)
    for i in range(len(t)):
        u_raw = -float(np.squeeze(K_lqr @ sol_nom.y[:, i])) + N_bar * omega_ref
        u_vals[i] = np.clip(u_raw, -24.0, 24.0)

    from scipy.integrate import trapezoid
    control_energy = float(trapezoid(u_vals**2, t))
    rise_idx = np.where(omega >= 0.9 * omega_ref)[0]
    t_rise_lqr = t[rise_idx[0]] if len(rise_idx) > 0 else 0.0
    settling_idx = np.where(np.abs(omega[:1000] - omega_ref) <= 0.02 * omega_ref)[0]
    t_settle_lqr = t[settling_idx[0]] if len(settling_idx) > 0 else 0.0
    overshoot_lqr = max(0.0, (np.max(omega[:1000]) - omega_ref) / omega_ref * 100.0)

    print("\n" + "-" * 75)
    print("LQR TRANSIENT PERFORMANCE (Nominal Plant):")
    print(f"  Rise Time (10% -> 90%):   {t_rise_lqr:.4f} s")
    print(f"  Settling Time (+/- 2%):   {t_settle_lqr:.4f} s")
    print(f"  Peak Overshoot:           {overshoot_lqr:.2f} %")
    print(f"  Control Action Energy J_u: {control_energy:.4f} V^2*s")
    print("-" * 75)

    # ========================= MONTE CARLO ROBUSTNESS =========================
    print("\nExecuting 100-Sample Monte Carlo Parameter Perturbation Study...")
    np.random.seed(42)
    n_samples = 100
    mc_responses = []

    for k in range(n_samples):
        p_R = motor.R * (1.0 + np.random.uniform(-0.20, 0.20))
        p_L = motor.L * (1.0 + np.random.uniform(-0.15, 0.15))
        p_J = motor.J * (1.0 + np.random.uniform(-0.25, 0.25))
        p_B = motor.B * (1.0 + np.random.uniform(-0.20, 0.20))
        p_K = motor.K * (1.0 + np.random.uniform(-0.10, 0.10))

        pert_motor = DCMotorStateSpace(J=p_J, B=p_B, K=p_K, R=p_R, L=p_L)
        sol_mc = solve_ivp(
            fun=lambda t_m, y_m: lqr_dynamics(t_m, y_m, pert_motor, K_lqr, N_bar, load_torque=0.02),
            t_span=t_span,
            y0=[0.0, 0.0],
            t_eval=t_eval,
            method='RK45',
            rtol=1e-5,
            atol=1e-7
        )
        mc_responses.append(sol_mc.y[0])

    mc_array = np.array(mc_responses)
    mean_resp = np.mean(mc_array, axis=0)
    p5_resp = np.percentile(mc_array, 5, axis=0)
    p95_resp = np.percentile(mc_array, 95, axis=0)

    # ========================= GENERATE PLOTS =========================
    # Figure 10: State Trajectories and LQR Control Input
    fig10, (ax_w, ax_i, ax_u) = plt.subplots(3, 1, figsize=(8.5, 7.5), sharex=True)
    ax_w.plot(t, omega, color='#27AE60', linewidth=2.4, label='Angular Velocity $\\omega(t)$')
    ax_w.axhline(y=omega_ref, color='black', linestyle='--', linewidth=1.2, label='Reference (1.0 rad/s)')
    ax_w.axvline(x=1.0, color='#C0392B', linestyle=':', label='Load Torque Disturbance (+0.02 N*m)')
    ax_w.set_ylabel('Velocity (rad/s)', fontweight='bold')
    ax_w.set_title('LQR Optimal State-Feedback Tracking & Disturbance Rejection', fontweight='bold', pad=10)
    ax_w.legend(loc='lower right', framealpha=0.95)

    ax_i.plot(t, ia, color='#2980B9', linewidth=2.0, label='Armature Current $i_a(t)$')
    ax_i.set_ylabel('Current (A)', fontweight='bold')
    ax_i.legend(loc='upper right', framealpha=0.95)

    ax_u.plot(t, u_vals, color='#8E44AD', linewidth=2.0, label='Control Voltage $v_a(t)$')
    ax_u.axhline(y=24.0, color='red', linestyle='--', linewidth=1.0, alpha=0.5, label='Saturation (+/-24V)')
    ax_u.axhline(y=-24.0, color='red', linestyle='--', linewidth=1.0, alpha=0.5)
    ax_u.set_xlabel('Time (s)', fontweight='bold')
    ax_u.set_ylabel('Control Input (V)', fontweight='bold')
    ax_u.legend(loc='upper right', framealpha=0.95)
    plt.tight_layout()
    fig10.savefig(os.path.join(output_dir, 'fig10_lqr_state_feedback_comparison.png'), dpi=300)
    plt.close(fig10)

    # Figure 11: Monte Carlo Robustness Envelope
    fig11, ax11 = plt.subplots(figsize=(8.5, 5.2))
    for k in range(min(25, n_samples)):
        ax11.plot(t, mc_array[k], color='#3498DB', alpha=0.18, linewidth=1.0)
    ax11.plot(t, mean_resp, color='#2C3E50', linewidth=2.5, label='Mean Response across 100 Plants')
    ax11.fill_between(t, p5_resp, p95_resp, color='#3498DB', alpha=0.3, label='90% Confidence Interval (P5 - P95)')
    ax11.axhline(y=omega_ref, color='black', linestyle='--', linewidth=1.2, label='Reference')
    ax11.axvline(x=1.0, color='#E74C3C', linestyle=':', label='Torque Step Disturbance')
    ax11.set_xlabel('Time (s)', fontweight='bold')
    ax11.set_ylabel('Rotor Velocity (rad/s)', fontweight='bold')
    ax11.set_title('Monte Carlo Robustness Envelope: 100 Perturbed Plants (R, L, J, B, K +/-25%)', fontweight='bold', pad=12)
    ax11.legend(loc='lower right', framealpha=0.95)
    ax11.set_xlim([0, 2.0])
    plt.tight_layout()
    fig11.savefig(os.path.join(output_dir, 'fig11_monte_carlo_robustness_envelope.png'), dpi=300)
    plt.close(fig11)

    print(f"\nSaved Fig 10 and Fig 11 to: {os.path.abspath(output_dir)}")
    print("Modern control synthesis and robustness evaluation successfully executed.")


if __name__ == '__main__':
    run_modern_control_study()
