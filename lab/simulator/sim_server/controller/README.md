# Crazyflie 2.1 BL Controller

This is a **logic-level port** of the Crazyflie 2.1 Brushless (CF2.1 BL) firmware controller from C to Python, designed for physics simulation with minimal sim2real gap. Control flow and units are kept consistent with the firmware, but the loop frequency and some gains are adapted for simulation timestep.

## Origin

Ported from [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware), specifically:
- **Target platform**: CF2.1 BL (Brushless)
- **Firmware source files**:
  - `src/modules/src/controller/attitude_pid_controller.c`
  - `src/modules/src/controller/position_controller_pid.c`
  - `src/modules/src/power_distribution_quadrotor.c`
  - `src/utils/src/pid.c`
  - `src/platform/interface/platform_defaults_cf21bl.h`

The control flow and motor mixing follow the firmware; gains are based on firmware defaults, but are slightly adjusted for the simulation attitude/rate loop (the firmware runs faster). Main changes:
1. Language: C → Python
2. Execution: Single-threaded → Vectorized (PyTorch)
3. Loop frequency: Attitude/rate 150HZ (firmware 250–500 Hz), corresponding gains slightly rescaled (see `config.py`)
4. Output: PWM commands → Body-frame force/torque for simulator (motor PWM still available for debugging)

## Architecture

The controller implements the same cascaded control structure as firmware:

```
Position PID → Velocity PID → Attitude PID → Rate PID → Power Distribution
```

**Control Modes** (currently available entry points):
- `ATTITUDE`: Direct roll/pitch setpoint + yaw rate
- `VELOCITY`: Body-frame velocity control
- `POSITION`: World-frame position control
(The enum includes `ATTITUDE_RATE` for firmware compatibility, but no public setter is provided yet.)

## Key Differences from Firmware

### What's Different

1. **Vectorized Operations**
   - All operations use PyTorch tensors
   - Supports batch simulation (multiple parallel environments)
   - No hardware-specific code (no motor drivers, sensors, etc.)

2. **Output Format**
   - Firmware: PWM commands (0-65535) to motor ESCs
   - This port: Body force (N) and torque (N⋅m) for physics simulator

3. **Runtime Flexibility**
   - PID gains can be tuned at runtime via `set_gains()`
   - Debug information extraction via `get_debug_info()`
   - Per-environment control mode switching

4. **Sim2Real Calibration**
   - Additional parameters for simulation calibration:
     - `MOTOR_TIME_CONSTANT`: Motor dynamics
     - `INERTIA_XX/YY/ZZ`: Inertia tensor
     - `DRAG_COEFFICIENT`: Aerodynamic drag
     - `ROTOR_DRAG_COEFFICIENT`: Rotor drag

### What's Preserved

* **PID gains** are based on firmware defaults, but attitude/rate loop gains are slightly reduced to match the 0.01 s timestep (see `config.py`).
- **Control flow** is identical (same function call order)
- **Motor mixing** matches `powerDistributionLegacy()` exactly
- **Units and conventions**:
  - Angles in degrees
  - Angular rates in deg/s
  - Position in meters
  - Velocity in m/s
  - Thrust in PWM scale (0-65535) internally
- **Attitude priority capping** when motors saturate
- **Yaw angle wrapping** for ±180° discontinuity
- **Derivative on measurement** (not error) to avoid derivative kick

## New Features

### 1. Batch Simulation
```python
controller = CrazyflieController(num_envs=16, device=torch.device("cuda"))
force, torque = controller.compute(state)  # [16, 3] tensors
```

### 2. Runtime Gain Tuning
Loops that can be tuned on the fly (keys → PID):
- `roll_rate`, `pitch_rate`, `yaw_rate`
- `roll`, `pitch`, `yaw`
- `vel_x`, `vel_y`, `vel_z`
- `pos_x`, `pos_y`, `pos_z`

Each accepts `kp`, `ki`, `kd`, optional `kff` and `i_limit`:
```python
controller.set_gains({
    "roll_rate": {"kp": 300.0, "ki": 500.0},
    "vel_z": {"kp": 28.0, "ki": 18.0},
    "pos_x": {"kp": 2.2, "ki": 0.15, "kd": 0.02},
})
```

### 3. Debug Information
```python
debug = controller.get_debug_info()
# Returns: control_mode, setpoints, motor_thrust, etc.
```

### 4. Per-Environment Control
```python
# Set position mode for specific environments
controller.set_position_setpoint(x, y, z, env_ids=env_ids)
```

## Usage

```python
from crazyflie_sim.controllers.cf_controller import CrazyflieController, config
import torch

# Initialize for 16 parallel environments
controller = CrazyflieController(
    num_envs=16,
    device=torch.device("cuda"),
    attitude_dt=config.ATTITUDE_UPDATE_DT,  # 0.01s (100Hz)
    position_dt=config.POSITION_UPDATE_DT,  # 0.01s (100Hz)
)

# Set control mode and setpoint
controller.set_position_setpoint(
    x=torch.tensor([1.0] * 16),
    y=torch.tensor([0.0] * 16),
    z=torch.tensor([0.5] * 16),
)

# Compute control output
state = {
    "position": ...,      # [16, 3] world position (m)
    "velocity": ...,      # [16, 3] world velocity (m/s)
    "attitude": ...,      # [16, 3] Euler angles (roll, pitch, yaw) in degrees
    "angular_velocity": ...,  # [16, 3] gyro rates (deg/s)
}
force, torque = controller.compute(state)
# force: [16, 3] body frame force (N)
# torque: [16, 3] body frame torque (N⋅m)
```

## Configuration

All parameters are in `config.py`, matching firmware defaults from:
- `platform_defaults_cf21bl.h`
- `stabilizer_types.h`

**Physical constants**:
- `ARM_LENGTH = 0.050 m` (CF2.1 BL)
- `CF_MASS = 0.0393 kg` (with guards, 350mAh battery)
- `THRUST_MAX = 0.2 N` per motor
- `THRUST2TORQUE = 0.00569 N⋅m/N`

**PID gains**: Derived from firmware defaults, with slight scaling for the simulation attitude/rate loop; see `config.py` for details.

## Sim2Real Considerations

Parameters that may need calibration between simulation and real hardware:
- `MOTOR_TIME_CONSTANT`: Motor response dynamics
- `INERTIA_XX/YY/ZZ`: Drone inertia tensor
- `DRAG_COEFFICIENT`: Aerodynamic drag
- `ROTOR_DRAG_COEFFICIENT`: Rotor drag (affects yaw damping)

These are documented in `config.py` with `NOTE:` comments.

## Implementation Notes

### Motor Layout
X-configuration (viewed from above, body ENU: +X forward, +Y left):
```
                 Front (+X)
            M4 (CW)     M1 (CCW)
                \\       /
                 \\     /
                  \\   /
     (left) <-- +Y  X
                  / \\
                 /   \\
            M3 (CCW)   M2 (CW)
                 Rear (-X)
```

### Control Flow
1. **Position/Velocity Control**: Computes desired roll/pitch/thrust
2. **Attitude Control**: Computes desired rates from attitude error
3. **Rate Control**: Computes actuator commands from rate error
4. **Power Distribution**: Converts to motor thrusts, caps, then to force/torque

### Units Convention
- **Angles**: Degrees (firmware convention)
- **Rates**: Degrees per second
- **Position**: Meters
- **Velocity**: Meters per second
- **Thrust**: PWM scale (0-65535) internally, converted to Newtons for output
- **Force/Torque**: Newtons and Newton-meters (for simulator)

## Files

- `controller.py`: Main controller combining all components
- `attitude_controller.py`: Cascaded attitude PID (outer: angle→rate, inner: rate→actuator)
- `position_controller.py`: Cascaded position PID (position→velocity→attitude+thrust)
- `power_distribution.py`: Motor mixing and force/torque conversion
- `pid.py`: Vectorized PID implementation
- `config.py`: All parameters matching firmware defaults

## License

This port maintains compatibility with the original firmware. Refer to the main project license for usage terms.
