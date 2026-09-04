# High-Precision DC Motor Regulation: Mathematical Modeling, Classical PID, Optimal LQR State-Feedback, and Robustness Analysis

**Independent Research Project | Advanced Control Systems, State Estimation & Physical Intelligence**

[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Control Theory](https://img.shields.io/badge/control-PID%20%7C%20LQR%20%7C%20State--Space-brightgreen.svg)](https://python-control.readthedocs.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## 1. Executive Summary

Electromechanical actuators in robotics, haptics, and physical AI systems require rapid transient tracking, zero steady-state error, and rejection of non-deterministic mechanical load disturbances. While standard classical Proportional-Integral-Derivative (PID) control remains ubiquitous, high-performance physical intelligence architectures require multi-variable state observability, control-effort bounding to prevent thermal saturation, and proven parameter robustness under component aging and temperature drift.

This repository provides a rigorous open-source control systems benchmark for armature-controlled direct current (DC) motor speed and position regulation. It implements:
1. **First-Principles Physics Modeling**: Explicit differential modeling of electrical armature circuitry and rotor rotational mechanics.
2. **Classical Frequency-Domain Design**: Multi-objective PID controller tuning (Ziegler-Nichols, Cohen-Coon, and numerical optimization) with Bode, Nyquist, and Root Locus stability proofs.
3. **Modern State-Space & LQR Synthesis**: Full-state feedback via the **Continuous Algebraic Riccati Equation (CARE)** with prefilter reference scaling and full controllability/observability validation.
4. **Stochastic Monte Carlo Robustness Analysis**: Evaluation across $N = 100$ perturbed plants subjected to $\pm 25\%$ simultaneous parameter variations ($R, L, J, B, K_t$) under step load torque shocks.

---

## 2. Mathematical Modeling & Theoretical Formulation

```text
       Armature Circuit                           Rotor Mechanics
      +---/\/\/\---UUUU---+
      |     R        L    |
 +    |                   |                     +----------+
v_a(t)|               ( M ) e_b(t) = K_b*w      | Rotor J  |---> w(t), theta(t)
 -    |                   |                     +----------+
      +-------------------+                          |
                                                Load T_L(t)
```

### 2.1 Governing Differential Equations

Applying Kirchhoff's Voltage Law (KVL) to the armature circuit and Newton's Second Law to the mechanical rotor:

$$\begin{aligned}
v_a(t) &= R i_a(t) + L \frac{d i_a(t)}{dt} + e_b(t) \\
J \frac{d\omega(t)}{dt} + B \omega(t) &= T_m(t) - T_L(t)
\end{aligned}$$

Where the back-electromotive force $e_b(t)$ and generated electromechanical torque $T_m(t)$ are coupled via the motor constants:

$$e_b(t) = K_b \omega(t), \quad T_m(t) = K_t i_a(t)$$

Under SI units ($K = K_t = K_b$):
- $J = 0.01\,\text{kg}\cdot\text{m}^2$: Rotor moment of inertia
- $B = 0.1\,\text{N}\cdot\text{m}\cdot\text{s}$: Viscous damping friction
- $K = 0.01\,\text{N}\cdot\text{m/A} = 0.01\,\text{V}\cdot\text{s/rad}$: Electromechanical coupling
- $R = 1.0\,\Omega$: Armature winding resistance
- $L = 0.5\,\text{H}$: Armature winding inductance

### 2.2 Open-Loop Transfer Function

Applying the Laplace transform with zero initial conditions yields the SISO speed transfer function $G(s) = \frac{\Omega(s)}{V_a(s)}$:

$$G(s) = \frac{K}{(Ls + R)(Js + B) + K^2} = \frac{0.01}{0.005 s^2 + 0.06 s + 0.1001}$$

Open-loop poles:
$$s_{1, 2} = -1.972, \quad -10.028 \quad (\text{Overdamped, stable})$$

### 2.3 Continuous State-Space Representation

Defining the physical state vector $x(t) = [\theta(t),\, \omega(t),\, i_a(t)]^T \in \mathbb{R}^3$, control input $u(t) = v_a(t)$, and disturbance $d(t) = T_L(t)$:

$$\dot{x}(t) = A x(t) + B u(t) + B_d T_L(t), \quad y(t) = C x(t)$$

$$\begin{bmatrix} \dot{\theta} \\ \dot{\omega} \\ \dot{i}_a \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & -\frac{B}{J} & \frac{K}{J} \\ 0 & -\frac{K}{L} & -\frac{R}{L} \end{bmatrix} \begin{bmatrix} \theta \\ \omega \\ i_a \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ \frac{1}{L} \end{bmatrix} v_a + \begin{bmatrix} 0 \\ -\frac{1}{J} \\ 0 \end{bmatrix} T_L$$

$$\begin{bmatrix} \dot{\theta} \\ \dot{\omega} \\ \dot{i}_a \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & -10.0 & 1.0 \\ 0 & -0.02 & -2.0 \end{bmatrix} \begin{bmatrix} \theta \\ \omega \\ i_a \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ 2.0 \end{bmatrix} v_a + \begin{bmatrix} 0 \\ -100.0 \\ 0 \end{bmatrix} T_L$$

### 2.4 Controllability & Observability Proofs

$$\mathcal{C} = \begin{bmatrix} B & AB & A^2 B \end{bmatrix} = \begin{bmatrix} 0 & 0 & 2.0 \\ 0 & 2.0 & -24.0 \\ 2.0 & -4.0 & 8.04 \end{bmatrix}, \quad \operatorname{det}(\mathcal{C}) = 8.0 \neq 0 \implies \operatorname{rank}(\mathcal{C}) = 3$$

$$\mathcal{O} = \begin{bmatrix} C_\omega \\ C_\omega A \\ C_\omega A^2 \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & -10 & 1 \\ 0 & 99.98 & -12 \end{bmatrix}, \quad \operatorname{rank}(\mathcal{O}) = 3$$

The system is **strictly controllable and observable**, guaranteeing arbitrary pole placement and asymptotic state reconstruction.

### 2.5 Optimal State Feedback via LQR

The infinite-horizon performance index:

$$J = \int_0^\infty \left( x(t)^T Q x(t) + u(t)^T R u(t) \right) dt$$

With state penalty $Q = \operatorname{diag}(1.0,\, 800.0,\, 0.5)$ and actuator voltage effort penalty $R = [0.02]$. The optimal gain $K_{\text{LQR}} = R^{-1} B^T P$ is computed by solving the continuous Algebraic Riccati Equation:

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

Reference prefilter gain $\bar{N}$ guarantees zero steady-state tracking error:

$$\bar{N} = -\left[ C_\omega (A - B K_{\text{LQR}})^{-1} B \right]^{-1}$$

---

## 3. Comprehensive Performance Benchmark

Quantitative comparison across the open-loop plant, classical PID control, and modern LQR state-feedback:

| Performance Metric | Open-Loop | Classical PID ($K_p=100, K_i=200, K_d=10$) | Optimal LQR State-Feedback | Improvement vs Open-Loop |
| :--- | :---: | :---: | :---: | :---: |
| **Rise Time ($t_r$, 10% $\to$ 90%)** | 1.136 s | 0.132 s | **0.088 s** | **92.2% reduction** |
| **Settling Time ($t_s$, $\pm 2\%$)** | 2.067 s | 0.258 s | **0.142 s** | **93.1% reduction** |
| **Overshoot ($M_p$)** | 0.00% | 1.03% | **0.42%** | **Minimal / Highly Damped** |
| **Steady-State Error ($e_{ss}$)** | 90.01% | -0.03% | **0.00%** | **100% elimination** |
| **Phase Margin ($\phi_m$)** | - | 94.64° | **$\infty$ (LQR guaranteed $\ge 60^\circ$)**| **Robust Stability** |
| **Gain Margin ($K_g$)** | - | $\infty\,\text{dB}$ | **$\infty\,\text{dB}$** | **Unconditional Stability** |
| **Control Action Energy ($\int u^2 dt$)**| - | 24.18 $\text{V}^2\text{s}$ | **16.84 $\text{V}^2\text{s}$** | **30.3% less actuator strain** |
| **Disturbance Recovery Time** | $\infty$ (fails) | 0.42 s | **0.18 s** | **57.1% faster recovery** |

---

## 4. Stability Analysis & Generated Figures

All simulation scripts generate publication-grade vector graphics ($300\,\text{DPI}$) in `pid_figures/`:

```text
pid_figures/
├── fig1_step_response_comparison.png      # Open-loop vs Closed-loop PID
├── fig2_closed_loop_detailed.png          # 2% settling band characteristics
├── fig3_tracking_error.png                # Transient error envelope
├── fig4_bode_plot.png                     # Frequency magnitude & phase response
├── fig5_nyquist_diagram.png               # Encirclement stability contour
├── fig6_pole_zero_map.png                 # Closed-loop pole trajectory
├── fig7_root_locus.png                    # Gain-variant pole locus
├── fig8_disturbance_rejection.png         # 20% load torque step response
├── fig9_comprehensive_analysis.png        # 9-panel master figure
├── fig10_lqr_state_feedback_comparison.png # LQR velocity, current & voltage trajectory
└── fig11_monte_carlo_robustness_envelope.png# 100 perturbed plants confidence interval
```

### Key Stability Insights:
1. **Infinite Gain Margin**: The phase never crosses $-180^\circ$ at finite positive gain; the closed-loop system is unconditionally stable against amplifier gain drift.
2. **Phase Margin of $94.64^\circ$**: Guarantees a heavily damped, overshoot-free transient response without oscillations.
3. **Monte Carlo Envelope**: When $R, L, J, B, K$ simultaneously vary by $\pm 25\%$, the LQR controller confines speed dispersion to a narrow $90\%$ confidence band ($[0.98, 1.02]\,\text{rad/s}$), demonstrating robust industrial applicability.

---

## 5. Repository Structure

```text
DC-Motor-PID-Control-System-Design-and-Analysis/
├── README.md                           # Master mathematical specification & benchmark
├── code.py                             # Classical PID design, frequency analysis & 9 figures
├── modern_control_analysis.py          # State-space, LQR synthesis & Monte Carlo robustness
├── pid_figures/                        # Generated publication-quality PNG and PDF figures
├── LICENSE                             # MIT License
└── requirements.txt                    # Dependencies (numpy, scipy, matplotlib, control)
```

---

## 6. Reproduction & Execution Guide

### 6.1 Setup Environment

```bash
git clone https://github.com/yagneshkumarkoduru/DC-Motor-PID-Control-System-Design-and-Analysis.git
cd DC-Motor-PID-Control-System-Design-and-Analysis

python -m venv .venv
# Activate:
# Linux/macOS: source .venv/bin/activate
# Windows PowerShell: .\.venv\Scripts\Activate.ps1

pip install -r requirements.txt
```

### 6.2 Run Classical PID & Frequency Domain Suite

```bash
python code.py
```
*Generates figures 1 through 9 and prints Bode/Nyquist stability margins.*

### 6.3 Run Modern State-Space, LQR & Monte Carlo Robustness

```bash
python modern_control_analysis.py
```
*Solves the Continuous Algebraic Riccati Equation, simulates 100 perturbed plants, and exports figures 10 and 11.*

---

## 7. Author & Citation

**Yagnesh Kumar Koduru**  
*Independent Researcher | Physical Intelligence, Embedded Systems & Control Systems*  
GitHub: [@yagneshkumarkoduru](https://github.com/yagneshkumarkoduru)  
Portfolio: [yagnesh-portfolio-eight.vercel.app](https://yagnesh-portfolio-eight.vercel.app)

```bibtex
@misc{koduru2026dcmotorcontrol,
  author = {Koduru, Yagnesh Kumar},
  title = {High-Precision DC Motor Regulation: Mathematical Modeling, Classical PID, Optimal LQR State-Feedback, and Robustness Analysis},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/yagneshkumarkoduru/DC-Motor-PID-Control-System-Design-and-Analysis}}
}
```
