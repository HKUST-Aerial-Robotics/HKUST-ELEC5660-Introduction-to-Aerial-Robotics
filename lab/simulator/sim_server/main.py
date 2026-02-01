#!/usr/bin/env python3
"""ELEC5660 single-drone simulator entrypoint."""
from __future__ import annotations

import argparse
import logging
import signal

from isaaclab.app import AppLauncher

from config import LOG_FORMAT, LOG_LEVEL, SimCfg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ELEC5660 Crazyflie Simulator",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


args = parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from sim.sim_manager import SimulationManager  # noqa: E402

_sim_manager: SimulationManager | None = None


def setup_logging() -> None:
    level = getattr(logging, LOG_LEVEL, logging.INFO)
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(LOG_FORMAT))

    root = logging.getLogger()
    root.setLevel(level)
    root.handlers.clear()
    root.addHandler(handler)


def signal_handler(sig, _frame):
    logging.info("Received signal %s, stopping simulator...", sig)
    if _sim_manager is not None:
        _sim_manager.stop()


def main() -> None:
    global _sim_manager

    setup_logging()
    logger = logging.getLogger(__name__)

    logger.info("=" * 60)
    logger.info("ELEC5660 Crazyflie Simulator")
    logger.info("=" * 60)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    cfg = SimCfg()
    logger.info("Configuration:\n%s", cfg)

    _sim_manager = SimulationManager(simulation_app=simulation_app, cfg=cfg)
    logger.info("Simulation initialized")

    _sim_manager.run()
    logger.info("Simulator shutdown complete")


if __name__ == "__main__":
    main()
