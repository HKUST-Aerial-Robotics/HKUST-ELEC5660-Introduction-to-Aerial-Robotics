"""
Simple performance micro-benchmarks for Crazyflie controller.

Measures controller.compute throughput for different parallel environment counts.
Standalone cf_controller repo usage examples:
    python -m perf.benchmark_controller --envs 1 8 32 128 --steps 1000
    python perf/benchmark_controller.py --envs 4 16 64 --steps 2000
"""
import argparse
import time
from typing import List, Dict, Any
import pathlib
import sys

import torch

# Run as a script without installation:
# add the directory containing the cf_controller package to sys.path
pkg_parent = pathlib.Path(__file__).resolve().parents[2]  # .../crazyflie_sim/controllers
sys.path.insert(0, str(pkg_parent))
from cf_controller.controller import CrazyflieController  # type: ignore
from cf_controller import config  # type: ignore


def build_state(num_envs: int, device: torch.device) -> Dict[str, torch.Tensor]:
    """Create a synthetic state tensor bundle matching controller expectations."""
    # Random but bounded values to avoid extreme setpoints
    position = torch.randn(num_envs, 3, device=device) * 0.1  # m
    velocity = torch.randn(num_envs, 3, device=device) * 0.5  # m/s
    attitude = torch.randn(num_envs, 3, device=device) * 10.0  # deg
    angular_velocity = torch.randn(num_envs, 3, device=device) * 50.0  # deg/s

    return {
        "position": position,
        "velocity": velocity,
        "attitude": attitude,
        "angular_velocity": angular_velocity,
    }


def benchmark_case(
    num_envs: int,
    device: torch.device,
    steps: int,
    warmup: int,
    attitude_dt: float,
    position_dt: float,
    mode: str,
) -> Dict[str, Any]:
    """Benchmark controller.compute for a single environment count."""
    controller = CrazyflieController(
        num_envs=num_envs,
        device=device,
        attitude_dt=attitude_dt,
        position_dt=position_dt,
    )

    # Configure control mode and setpoints
    if mode == "attitude":
        controller.set_attitude_setpoint(
            roll=torch.zeros(num_envs, device=device),
            pitch=torch.zeros(num_envs, device=device),
            yaw_rate=torch.zeros(num_envs, device=device),
            thrust=torch.full((num_envs,), 0.5, device=device),
        )
    elif mode == "velocity":
        controller.set_velocity_setpoint(
            vx=torch.zeros(num_envs, device=device),
            vy=torch.zeros(num_envs, device=device),
            vz=torch.zeros(num_envs, device=device),
            yaw_rate=torch.zeros(num_envs, device=device),
        )
        controller.thrust_setpoint = torch.full((num_envs,), 0.5 * 65535.0, device=device)
    elif mode == "position":
        controller.set_position_setpoint(
            x=torch.zeros(num_envs, device=device),
            y=torch.zeros(num_envs, device=device),
            z=torch.full((num_envs,), 0.5, device=device),
            yaw=torch.zeros(num_envs, device=device),
        )
        controller.thrust_setpoint = torch.full((num_envs,), 0.5 * 65535.0, device=device)
    else:
        raise ValueError(f"Unsupported mode: {mode}")

    state = build_state(num_envs, device)

    # Warmup (GPU kernels, cache effects)
    with torch.no_grad():
        for _ in range(warmup):
            controller.compute(state)
        if device.type == "cuda":
            torch.cuda.synchronize(device)

        start = time.perf_counter()
        for _ in range(steps):
            controller.compute(state)
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        elapsed = time.perf_counter() - start

    steps_per_sec = steps / elapsed
    env_steps_per_sec = steps * num_envs / elapsed
    ms_per_step = elapsed / steps * 1e3
    us_per_env_step = elapsed / (steps * num_envs) * 1e6

    return {
        "envs": num_envs,
        "steps": steps,
        "elapsed_s": elapsed,
        "steps_per_sec": steps_per_sec,
        "env_steps_per_sec": env_steps_per_sec,
        "ms_per_step": ms_per_step,
        "us_per_env_step": us_per_env_step,
    }


def format_result(res: Dict[str, Any]) -> str:
    """Format a single benchmark row."""
    return (
        f"{res['envs']:>5d} | "
        f"{res['steps_per_sec']:>10.1f} | "
        f"{res['env_steps_per_sec']/1e3:>10.1f}k | "
        f"{res['ms_per_step']:>8.3f} | "
        f"{res['us_per_env_step']:>10.2f}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Crazyflie controller perf benchmarks")
    parser.add_argument(
        "--envs",
        type=int,
        nargs="+",
        default=[1, 8, 32, 128],
        help="List of parallel environment counts to benchmark",
    )
    parser.add_argument("--steps", type=int, default=1000, help="Timed iterations per benchmark case")
    parser.add_argument("--warmup", type=int, default=100, help="Warmup iterations per case")
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        choices=["cpu", "cuda"],
        help="Device to run benchmarks on",
    )
    parser.add_argument("--attitude-dt", type=float, default=None, help="Override attitude_dt (seconds)")
    parser.add_argument("--position-dt", type=float, default=None, help="Override position_dt (seconds)")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["attitude", "velocity", "position"],
        default="attitude",
        help="Control mode to benchmark (default: attitude)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    device = torch.device(args.device if args.device == "cpu" or torch.cuda.is_available() else "cpu")

    attitude_dt = args.attitude_dt if args.attitude_dt is not None else config.ATTITUDE_UPDATE_DT
    position_dt = args.position_dt if args.position_dt is not None else config.POSITION_UPDATE_DT

    print(f"Running on device: {device} | mode: {args.mode}")
    if device.type == "cuda":
        print(f"CUDA device: {torch.cuda.get_device_name(device)}")

    header = "envs |   steps/s | envSteps/s |  ms/step |  us/env-step"
    print(header)
    print("-" * len(header))

    results: List[Dict[str, Any]] = []
    for envs in args.envs:
        res = benchmark_case(
            num_envs=envs,
            device=device,
            steps=args.steps,
            warmup=args.warmup,
            attitude_dt=attitude_dt,
            position_dt=position_dt,
            mode=args.mode,
        )
        results.append(res)
        print(format_result(res))


if __name__ == "__main__":
    main()
