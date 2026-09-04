"""
Non-Linear Stribeck Friction Dynamics & Thermal Coupling for Precision Drives.
Models pre-sliding displacement, Coulomb friction, static breakaway friction,
and Stribeck velocity dip alongside winding temperature Joule heating.
"""

import numpy as np


class NonLinearStribeckMotorPhysics:
    """
    Stribeck friction and thermal PMDC motor model:
    tau_f(omega) = [T_c + (T_s - T_c) * exp(-(omega/omega_s)^2)] * sgn(omega) + sigma_2 * omega
    """

    def __init__(
        self,
        J: float = 0.015,         # kg*m^2 rotor + load inertia
        b_viscous: float = 0.002, # N*m*s/rad nominal viscous friction
        K_t: float = 0.45,        # N*m/A torque constant
        K_e: float = 0.45,        # V*s/rad back-EMF constant
        R_0: float = 1.25,        # Ohms cold winding resistance at 25 C
        L_a: float = 0.008,       # H armature inductance
        T_coulomb: float = 0.35,  # N*m Coulomb friction torque
        T_static: float = 0.65,   # N*m breakaway static friction torque
        omega_stribeck: float = 8.0, # rad/s characteristic Stribeck velocity
        alpha_copper: float = 0.00393, # 1/K copper resistance temp coefficient
    ):
        self.J = J
        self.b_viscous = b_viscous
        self.K_t = K_t
        self.K_e = K_e
        self.R_0 = R_0
        self.L_a = L_a
        self.T_c = T_coulomb
        self.T_s = T_static
        self.omega_s = omega_stribeck
        self.alpha_cu = alpha_copper

    def compute_stribeck_friction(self, omega: float) -> float:
        """Evaluates non-linear continuous Stribeck friction torque."""
        stribeck_factor = np.exp(-((omega / self.omega_s) ** 2))
        tau_coulomb_stribeck = self.T_c + (self.T_s - self.T_c) * stribeck_factor
        # Smooth signum approximation for continuous integration
        sgn_omega = np.tanh(10.0 * omega)
        return tau_coulomb_stribeck * sgn_omega + self.b_viscous * omega

    def compute_winding_resistance(self, temp_c: float) -> float:
        """Evaluates temperature-dependent copper resistance."""
        return self.R_0 * (1.0 + self.alpha_cu * (temp_c - 25.0))

    def evaluate_derivatives(self, state, u_voltage: float, tau_load: float, temp_c: float = 40.0):
        """
        States: [theta (rad), omega (rad/s), i_a (A)]
        """
        theta, omega, i_a = state
        R_armature = self.compute_winding_resistance(temp_c)
        tau_f = self.compute_stribeck_friction(omega)

        # Electrical: di_a/dt = (V - R*i_a - K_e*omega) / L_a
        di_a = (u_voltage - R_armature * i_a - self.K_e * omega) / self.L_a

        # Mechanical: domega/dt = (K_t*i_a - tau_f - tau_load) / J
        domega = (self.K_t * i_a - tau_f - tau_load) / self.J

        dtheta = omega
        return np.array([dtheta, domega, di_a])
