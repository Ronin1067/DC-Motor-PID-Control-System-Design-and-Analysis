"""
High-Gain Linear Extended State Observer (LESO) for Precision Motor Drives.
Estimates angular velocity and lumped total disturbance (unmodeled friction,
load torque, back-EMF harmonics) with sub-2 ms reconstruction bandwidth.
"""

import numpy as np


class HighGainLESO:
    """
    3rd-order LESO for mechanical subsystem:
    y = theta
    dtheta/dt = omega
    domega/dt = b0 * u + f_total
    df_total/dt = h(t)
    Observer poles placed at -omega_o:
    L = [3*omega_o, 3*omega_o^2, omega_o^3]^T
    """

    def __init__(self, b0: float, omega_o: float = 450.0):
        self.b0 = b0
        self.omega_o = omega_o
        self.beta1 = 3.0 * omega_o
        self.beta2 = 3.0 * (omega_o ** 2)
        self.beta3 = 1.0 * (omega_o ** 3)

        # Observer states: [z1_hat (pos), z2_hat (vel), z3_hat (lumped disturbance)]
        self.z = np.zeros(3)

    def reset(self, initial_pos: float = 0.0, initial_vel: float = 0.0):
        self.z = np.array([initial_pos, initial_vel, 0.0])

    def update(self, y_measured_pos: float, u_input: float, dt: float):
        """Discrete-time Euler/RK2 update of extended state observer."""
        e = self.z[0] - y_measured_pos

        dz0 = self.z[1] - self.beta1 * e
        dz1 = self.z[2] - self.beta2 * e + self.b0 * u_input
        dz2 = -self.beta3 * e

        self.z[0] += dz0 * dt
        self.z[1] += dz1 * dt
        self.z[2] += dz2 * dt

        return {
            "pos_est": self.z[0],
            "vel_est": self.z[1],
            "disturbance_est": self.z[2]
        }
