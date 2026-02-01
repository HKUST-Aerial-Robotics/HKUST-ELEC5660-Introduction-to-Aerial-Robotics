"""
Crazyflie 2.1 BL firmware-compatible controller for IsaacLab simulation.

This module provides a complete controller stack matching the CF2.1 BL firmware:
- Cascaded PID control (position -> velocity -> attitude -> rate)
- Power distribution (motor mixing)
- Vectorized torch operations for batch simulation

Usage:
    from crazyflie_sim.controllers.cf_controller import CrazyflieController, config

    controller = CrazyflieController(num_envs=16, device=device)
    force, torque = controller.compute(state)
"""
from .controller import CrazyflieController, ControlMode
from .attitude_controller import AttitudeController
from .position_controller import PositionController
from .power_distribution import PowerDistribution
from .pid import PID
from . import config

__all__ = [
    "CrazyflieController",
    "ControlMode",
    "AttitudeController",
    "PositionController",
    "PowerDistribution",
    "PID",
    "config",
]

