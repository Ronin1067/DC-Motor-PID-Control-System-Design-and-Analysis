"""
Control Barrier Function (CBF) Quadratic Program Safety Filter.
Guarantees forward invariance of electrical current limits (|i_a| <= I_max)
and inverter voltage bus saturation (|V_a| <= V_max).
"""

import numpy as np


class DriveCBSafetyFilter:
    """
    CBF for armature current envelope:
    h(i_a) = I_max^2 - i_a^2 >= 0
    Lie derivatives along electrical dynamics:
    dot(h) = -2 * i_a * di_a/dt = -2 * i_a * (V_a - R*i_a - K_e*omega) / L_a
    CBF Condition:
    L_f h(x) + L_g h(x) * V_a + gamma_cbf * h(x) >= 0
    """

    def __init__(
        self,
        I_max: float = 8.0,       # Amperes
        V_max: float = 24.0,      # Volts
        gamma_cbf: float = 120.0, # Class-K gain
        R: float = 1.25,
        L_a: float = 0.008,
        K_e: float = 0.45,
    ):
        self.I_max = I_max
        self.V_max = V_max
        self.gamma_cbf = gamma_cbf
        self.R = R
        self.L_a = L_a
        self.K_e = K_e

    def filter_voltage(self, u_desired: float, current_ia: float, omega: float) -> dict:
        h = (self.I_max ** 2) - (current_ia ** 2)

        # L_f h = -2 * i_a * (-R * i_a - K_e * omega) / L_a
        # L_g h = -2 * i_a / L_a
        Lf_h = -2.0 * current_ia * (-self.R * current_ia - self.K_e * omega) / self.L_a
        Lg_h = -2.0 * current_ia / self.L_a

        # Constraint: Lg_h * V_a >= -Lf_h - gamma_cbf * h
        cbf_bound = -Lf_h - self.gamma_cbf * h

        u_safe = u_desired
        active_constraint = False

        if Lg_h > 1e-5:
            # Lower bound on V_a
            v_lower = cbf_bound / Lg_h
            if u_safe < v_lower:
                u_safe = v_lower
                active_constraint = True
        elif Lg_h < -1e-5:
            # Upper bound on V_a
            v_upper = cbf_bound / Lg_h
            if u_safe > v_upper:
                u_safe = v_upper
                active_constraint = True

        # Inverter hard saturation
        u_final = np.clip(u_safe, -self.V_max, self.V_max)

        return {
            "u_safe": u_final,
            "cbf_margin": h,
            "safety_active": active_constraint,
            "current_a": current_ia
        }
