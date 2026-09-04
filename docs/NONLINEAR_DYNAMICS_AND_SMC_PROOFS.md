# Mathematical Derivations, Non-Linear Dynamics & Sliding Mode Proofs

**Precision-Drive-Dynamics: High-Precision Electro-Mechanical Drive Systems**  
*Esthien Labs Technical Report | Control Theory & Mechatronics Series*

---

## 1. Mathematical Formulation of Motor Electro-Mechanical Dynamics

The dynamics of a permanent-magnet DC/BLDC drive with non-linear friction and lumped parameter variation are described by coupled state-space equations:

$$\begin{aligned}
\frac{d\theta}{dt} &= \omega \\
J \frac{d\omega}{dt} &= K_t i_a - \tau_f(\omega) - \tau_L(t) + \Delta f(t) \\
L_a \frac{di_a}{dt} &= V_a - R_a(T) i_a - K_e \omega - V_{\text{deadtime}}
\end{aligned}$$

where:
* $\theta(t), \omega(t)$: Rotor angular position ($\text{rad}$) and velocity ($\text{rad/s}$)
* $i_a(t)$: Armature current ($\text{A}$)
* $V_a(t)$: Applied inverter bridge terminal voltage ($\text{V}$)
* $J = 0.015\text{ kg}\cdot\text{m}^2$: Rotor and reflected load mass moment of inertia
* $K_t = 0.45\text{ N}\cdot\text{m/A}$, $K_e = 0.45\text{ V}\cdot\text{s/rad}$: Electromechanical torque and back-EMF constants
* $R_a(T)$: Armature winding resistance with temperature dependence ($R_0 = 1.25\,\Omega$)
* $L_a = 8.0\text{ mH}$: Armature winding inductance
* $\tau_L(t)$: External physical load torque disturbance ($\text{N}\cdot\text{m}$)
* $\tau_f(\omega)$: Non-linear friction torque

### 1.1 Non-Linear Stribeck Friction Dynamics
To capture pre-sliding displacement, static breakaway, and boundary-to-fluid transition:

$$\tau_f(\omega) = \left[ T_c + (T_s - T_c) \exp\left(-\left(\frac{\omega}{\omega_s}\right)^2\right) \right] \operatorname{sgn}(\omega) + b_{\text{visc}} \omega$$

where $T_c = 0.25\text{ N}\cdot\text{m}$ is Coulomb friction, $T_s = 0.55\text{ N}\cdot\text{m}$ is static breakaway torque, $\omega_s = 6.0\text{ rad/s}$ is characteristic Stribeck velocity threshold, and $b_{\text{visc}} = 0.08\text{ N}\cdot\text{m}\cdot\text{s/rad}$ is viscous damping.

### 1.2 Thermal Resistance Coupling
Armature copper winding resistance varies with operating temperature $T$:

$$R_a(T) = R_0 \left[ 1 + \alpha_{\text{Cu}} (T - T_0) \right]$$

with $\alpha_{\text{Cu}} = 0.00393\text{ K}^{-1}$ at $T_0 = 298.15\text{ K}$ ($25^\circ\text{C}$).

---

## 2. High-Gain Linear Extended State Observer (LESO)

Defining the mechanical acceleration state equation:

$$\dot{\omega} = b_0 u + f_{\text{total}}(\omega, i_a, t)$$

where $b_0 = \frac{K_t}{J R_a}$ and $f_{\text{total}} = -\frac{b_{\text{visc}}}{J} \omega - \frac{\tau_f(\omega) + \tau_L(t)}{J} + \Delta f$. We design a third-order LESO to estimate rotor position $z_1 = \theta$, velocity $z_2 = \omega$, and lumped unmodeled dynamics $z_3 = f_{\text{total}}$:

$$\begin{aligned}
\dot{\hat{z}}_1 &= \hat{z}_2 - \beta_1 (\hat{z}_1 - y) \\
\dot{\hat{z}}_2 &= \hat{z}_3 - \beta_2 (\hat{z}_1 - y) + b_0 u \\
\dot{\hat{z}}_3 &= -\beta_3 (\hat{z}_1 - y)
\end{aligned}$$

Assigning observer poles at $-\omega_o$ via symmetric bandwidth parameterization:

$$\beta_1 = 3 \omega_o, \quad \beta_2 = 3 \omega_o^2, \quad \beta_3 = \omega_o^3$$

### Lemma 1 (LESO Estimation Error Boundedness)
> If the disturbance rate of change satisfies $|\dot{f}_{\text{total}}| \le M_d < \infty$, then the observer error vector $\tilde{z} = z - \hat{z}$ is globally uniformly ultimately bounded (UUB), with steady-state error bounded by $\|\tilde{z}\| \le \frac{M_d}{\omega_o} \kappa$ for some positive constant $\kappa > 0$.

---

## 3. Super-Twisting Second-Order Sliding Mode Control (2-SMC)

Define the velocity tracking error manifold:

$$s(t) = \omega_{\text{ref}}(t) - \omega(t)$$

The continuous Super-Twisting control algorithm synthesizes:

$$u_{\text{STA}}(t) = k_1 |s|^{1/2} \operatorname{sgn}(s) + v(t), \quad \dot{v}(t) = k_2 \operatorname{sgn}(s)$$

### Theorem 1 (Finite-Time Convergence to Sliding Manifold)
> **Theorem 1.** Assume the derivative of the total perturbation is bounded such that $|\dot{f}_{\text{total}}| \le L$. If the Super-Twisting gains satisfy:
>
> $$k_1 > 2 \sqrt{L}, \quad k_2 > \frac{k_1 (L + 4 L^2)}{2 (k_1 - 2 \sqrt{L})}$$
>
> then the sliding manifold $s(t) = 0$ and its derivative $\dot{s}(t) = 0$ are reached in finite time $T_{\text{reach}} \le \frac{2 V_0(s(0), v(0))^{1/2}}{\gamma_{\min}}$, without high-frequency control chattering.

**Proof.** Consider the quadratic Moreno-Osorio Lyapunov function candidate:

$$V(x) = \zeta^T P \zeta, \quad \zeta = \begin{bmatrix} |s|^{1/2} \operatorname{sgn}(s) \\ v \end{bmatrix}, \quad P = \frac{1}{2} \begin{bmatrix} 4 k_2 + k_1^2 & -k_1 \\ -k_1 & 2 \end{bmatrix}$$

Since $P$ is positive definite ($|P| = 4 k_2 > 0$), $V(x)$ is positive definite. Time-differentiating along the closed-loop trajectories yields:

$$\dot{V}(x) = -\frac{1}{|s|^{1/2}} \zeta^T Q \zeta \le -\lambda_{\min}(Q) \|\zeta\|^2 \frac{1}{|s|^{1/2}} \le -\gamma V^{1/2}(x)$$

where $Q$ is strictly positive definite under the gain conditions. Integrating $\dot{V} \le -\gamma V^{1/2}$ yields finite-time convergence $T_{\text{reach}} \le \frac{2 V^{1/2}(0)}{\gamma}$. $\blacksquare$

---

## 4. Control Barrier Function (CBF) Safety Filter

To safeguard against inverter overvoltage and winding burnout, we define the current safety set:

$$\mathcal{C}_i = \left\{ i_a \in \mathbb{R} \mid h(i_a) = I_{\max}^2 - i_a^2 \ge 0 \right\}$$

The CBF condition requires:

$$L_f h(x) + L_g h(x) V_a + \gamma_{\text{cbf}} h(i_a) \ge 0$$

where:

$$L_f h = -2 i_a \left( \frac{-R_a i_a - K_e \omega}{L_a} \right), \quad L_g h = -\frac{2 i_a}{L_a}$$

The instantaneous Quadratic Program:

$$\min_{V_a} \frac{1}{2} \|V_a - V_{\text{STA}}\|^2 \quad \text{s.t.} \quad L_g h(x) V_a \ge -L_f h(x) - \gamma_{\text{cbf}} h(x), \quad |V_a| \le V_{\max}$$

strictly projects the SMC action to ensure forward invariance of the electrical thermal boundary.
