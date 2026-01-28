from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class QuadParams:
    """Physical parameters for a 500 g quadrotor."""

    mass: float = 0.5
    grav: float = 9.81
    I: np.ndarray = field(default_factory=lambda: np.diag([2.32e-3, 2.32e-3, 4.0e-3]))
    maxangle: float = np.deg2rad(40.0)
    kforce: float = 6.11e-8
    kmoment: float = 1.5e-9
    armlength: float = 0.175

    invI: np.ndarray = field(init=False)
    maxF: float = field(init=False)
    minF: float = field(init=False)
    FM_omega2: np.ndarray = field(init=False)
    omega2_FM: np.ndarray = field(init=False)
    maxomega: float = field(init=False)
    minomega: float = field(init=False)

    def __post_init__(self) -> None:
        self.invI = np.linalg.inv(self.I)
        self.maxF = 2.5 * self.mass * self.grav
        self.minF = 0.05 * self.mass * self.grav
        self.FM_omega2 = np.array(
            [
                [self.kforce, self.kforce, self.kforce, self.kforce],
                [0.0, self.armlength * self.kforce, 0.0, -self.armlength * self.kforce],
                [-self.armlength * self.kforce, 0.0, self.armlength * self.kforce, 0.0],
                [self.kmoment, -self.kmoment, self.kmoment, -self.kmoment],
            ]
        )
        self.omega2_FM = np.linalg.inv(self.FM_omega2)
        self.maxomega = np.sqrt(self.maxF / (4.0 * self.kforce))
        self.minomega = np.sqrt(self.minF / (4.0 * self.kforce))
