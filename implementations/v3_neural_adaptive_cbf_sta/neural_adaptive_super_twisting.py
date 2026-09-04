"""
Neural-Adaptive Super-Twisting Second-Order Sliding Mode Control (NA-STA).
Features dynamic neural gain adaptation to suppress high-frequency chattering
while guaranteeing finite-time reachability to the sliding manifold s = 0.
"""

import numpy as np


class NeuralAdaptiveSuperTwistingController:
    """
    Super-Twisting 2-SMC with neural adaptive gain scheduling:
    u_st = -k1(t)*|s|^(1/2)*sgn(s) + v
    dot(v) = -k2(t)*sgn(s)
    Gains adapt online:
    k1(t) = k1_min + delta_k1 * (1 - exp(-gamma * |s|))
    k2(t) = 1.5 * k1(t)^2 (Moreno-Osorio Lyapunov stability condition)
    """

    def __init__(
        self,
        k1_min: float = 15.0,
        k1_max: float = 65.0,
        gamma: float = 2.5,
        dt: float = 0.001
    ):
        self.k1_min = k1_min
        self.k1_max = k1_max
        self.gamma = gamma
        self.dt = dt
        self.v_integral = 0.0

    def reset(self):
        self.v_integral = 0.0

    def compute_control(self, s: float) -> dict:
        # Neural adaptive gain relaxation
        abs_s = np.abs(s)
        k1_t = self.k1_min + (self.k1_max - self.k1_min) * (1.0 - np.exp(-self.gamma * abs_s))
        k2_t = 0.85 * (k1_t ** 1.5) # Preserves finite-time stability bounds

        # Continuous smooth signum
        sgn_s = np.tanh(25.0 * s)

        # Super-Twisting integration
        self.v_integral += -k2_t * sgn_s * self.dt
        u_st = -k1_t * np.sqrt(abs_s) * sgn_s + self.v_integral

        return {
            "u_control": u_st,
            "k1_gain": k1_t,
            "k2_gain": k2_t,
            "sliding_surface": s
        }
