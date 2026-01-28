from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .controller import Controller
from .dynamics import quad_eom
from .math_utils import R_to_quaternion, quaternion_to_R, rot_to_rpy_zxy, wrap_to_pi, ypr_to_R
from .model import QuadParams


@dataclass
class SimulationResult:
    times: np.ndarray
    states: np.ndarray
    desired: np.ndarray
    rpy: np.ndarray
    rmse_pos: float | None = None
    rmse_vel: float | None = None
    rmse_yaw_deg: float | None = None
    smoothness: float | None = None


def initial_state() -> np.ndarray:
    x0 = np.zeros(13)
    yaw0 = -30.0 * np.pi / 180.0
    pitch0 = -30.0 * np.pi / 180.0
    roll0 = -30.0 * np.pi / 180.0
    quat0 = R_to_quaternion(ypr_to_R(np.array([yaw0, pitch0, roll0])).T)
    x0[6:10] = quat0
    return x0


def rk4_step(
    f,
    t: float,
    y: np.ndarray,
    dt: float,
    F: float,
    M: np.ndarray,
    Fd: np.ndarray,
    params: QuadParams,
) -> np.ndarray:
    k1 = f(t, y, F, M, Fd, params)
    k2 = f(t + dt / 2.0, y + dt * k1 / 2.0, F, M, Fd, params)
    k3 = f(t + dt / 2.0, y + dt * k2 / 2.0, F, M, Fd, params)
    k4 = f(t + dt, y + dt * k3, F, M, Fd, params)
    return y + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


class SimulationEngine:
    """Stateful simulator that can be stepped for live visualization."""

    def __init__(
        self,
        trajectory_fn,
        params: QuadParams | None = None,
        t_final: float = 25.0,
        t_step: float = 0.002,
        control_step: float = 0.01,
        fnoise: float = 1.0,
        seed: int | None = None,
    ) -> None:
        self.trajectory_fn = trajectory_fn
        self.params = params or QuadParams()
        self.t_final = float(t_final)
        self.t_step = float(t_step)
        self.control_step = float(control_step)
        self.fnoise = float(fnoise)
        self.seed = seed

        self.steps_per_control = int(round(self.control_step / self.t_step))
        if abs(self.steps_per_control * self.t_step - self.control_step) > 1e-9:
            raise ValueError("control_step must be an integer multiple of t_step")

        self._rng = np.random.default_rng(self.seed)
        self.controller = Controller(self.params)
        self.reset()

    def reset(self) -> None:
        self.t = 0.0
        self.state = initial_state()
        self.F = self.params.mass * self.params.grav
        self.M = np.zeros(3)
        self._times: list[float] = []
        self._states: list[np.ndarray] = []
        self._desired: list[np.ndarray] = []
        self._rpy: list[np.ndarray] = []
        self._forces: list[float] = []
        self._moments: list[np.ndarray] = []
        self.done = False
        self.controller.reset()
        self._rng = np.random.default_rng(self.seed)

    def step(self, controls: int = 1) -> None:
        if self.done:
            return

        for _ in range(max(1, int(controls))):
            if self.t >= self.t_final - 1e-12:
                self.done = True
                break
            Fd = self._rng.normal(0.0, self.fnoise, 3)
            for _ in range(self.steps_per_control):
                self.state = rk4_step(
                    quad_eom,
                    self.t,
                    self.state,
                    self.t_step,
                    self.F,
                    self.M,
                    Fd,
                    self.params,
                )
                self.t += self.t_step

            des = self.trajectory_fn(self.t, self.state)
            self.F, self.M = self.controller(self.t, self.state, des)

            self._times.append(self.t)
            self._states.append(self.state.copy())
            self._desired.append(des.copy())
            self._rpy.append(rot_to_rpy_zxy(quaternion_to_R(self.state[6:10])))
            self._forces.append(self.F)
            self._moments.append(self.M.copy())

            if self.t >= self.t_final - 1e-12:
                self.done = True
                break

    def result(self, compute_metrics: bool = True) -> SimulationResult:
        if self._times:
            times = np.asarray(self._times)
            states = np.vstack(self._states)
            desired = np.vstack(self._desired)
            rpy = np.vstack(self._rpy)
        else:
            times = np.zeros(0)
            states = np.zeros((0, 13))
            desired = np.zeros((0, 11))
            rpy = np.zeros((0, 3))

        if compute_metrics:
            rmse_pos, rmse_vel, rmse_yaw_deg = compute_rmse(states, desired, rpy)
            forces = np.array(self._forces)
            moments = np.array(self._moments)
            smoothness = compute_smoothness(forces, moments)
        else:
            rmse_pos, rmse_vel, rmse_yaw_deg = None, None, None
            smoothness = None

        return SimulationResult(
            times=times,
            states=states,
            desired=desired,
            rpy=rpy,
            rmse_pos=rmse_pos,
            rmse_vel=rmse_vel,
            rmse_yaw_deg=rmse_yaw_deg,
            smoothness=smoothness,
        )


def run_simulation(
    trajectory_fn,
    params: QuadParams | None = None,
    t_final: float = 25.0,
    t_step: float = 0.002,
    control_step: float = 0.01,
    fnoise: float = 1.0,
    seed: int | None = None,
) -> SimulationResult:
    engine = SimulationEngine(
        trajectory_fn=trajectory_fn,
        params=params,
        t_final=t_final,
        t_step=t_step,
        control_step=control_step,
        fnoise=fnoise,
        seed=seed,
    )
    while not engine.done:
        engine.step()
    return engine.result()


def compute_rmse(states: np.ndarray, desired: np.ndarray, rpy_hist: np.ndarray) -> tuple[float, float, float]:
    if states.size == 0:
        return 0.0, 0.0, 0.0

    pos_err = states[:, 0:3] - desired[:, 0:3]
    vel_err = states[:, 3:6] - desired[:, 3:6]
    yaw_err = wrap_to_pi(rpy_hist[:, 2] - desired[:, 9])

    rmse_pos = float(np.sqrt(np.mean(pos_err**2)))
    rmse_vel = float(np.sqrt(np.mean(vel_err**2)))
    rmse_yaw = float(np.sqrt(np.mean(yaw_err**2)))
    return rmse_pos, rmse_vel, np.degrees(rmse_yaw)


def compute_smoothness(forces: np.ndarray, moments: np.ndarray) -> float:
    """Compute smoothness metric based on control input variations.

    Lower values indicate smoother control (less jerk in control inputs).

    Args:
        forces: Array of thrust forces over time (N,)
        moments: Array of moment vectors over time (N, 3)

    Returns:
        Smoothness score (lower is better)
    """
    if forces.size < 2:
        return 0.0

    # Compute differences (control rate of change)
    dF = np.diff(forces)
    dM = np.diff(moments, axis=0)

    # Compute RMS of control derivatives
    rms_dF = np.sqrt(np.mean(dF**2))
    rms_dM = np.sqrt(np.mean(dM**2))

    # Combined smoothness score (normalized)
    smoothness = rms_dF + rms_dM * 10.0  # Scale moment changes
    return float(smoothness)
