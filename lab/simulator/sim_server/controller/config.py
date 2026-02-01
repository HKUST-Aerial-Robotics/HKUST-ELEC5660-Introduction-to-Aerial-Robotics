"""
Crazyflie 2.1 Brushless (CF2.1 BL) firmware-compatible default parameters.

Only parameters actually consumed by the controller stack are kept here.
Anything unused in the current code path was removed to avoid configuration
bloat and accidental drift.
"""

# =============================================================================
# Physical constants
# =============================================================================
# Gravity (m/s^2)
GRAVITY = 9.81

# Drone mass (kg)
# NOTE: This may vary with battery and attachments. Measure for sim2real.
# CF_MASS = 0.0393  # kg (CF2.1 BL with guards, 350mAh battery, LH deck)
CF_MASS = 0.050  # kg

# Arm length from center to motor (m). Should match the real airframe.
ARM_LENGTH = 0.050  # CF2.1 BL uses 0.050m, standard CF2 uses 0.046m

# Thrust per motor (N)
THRUST_MIN = 0.02136263065537499
THRUST_MAX = 0.4

# Thrust-to-torque coefficient (N*m / N)
THRUST2TORQUE = 0.00569278844371417

# Inertia tensor diagonal [Ixx, Iyy, Izz] (kg*m^2)
# NOTE: Critical for attitude dynamics. Measure or estimate for sim2real.
# These are typical values for CF2.1 BL - may need calibration
INERTIA_XX = 2.16e-5  # kg*m^2
INERTIA_YY = 2.16e-5  # kg*m^2
INERTIA_ZZ = 4.33e-5  # kg*m^2

# =============================================================================
# Timing
# =============================================================================

# Controller update timesteps
ATTITUDE_UPDATE_DT = 1.0 / 100.0  # 100 Hz attitude loop
POSITION_UPDATE_DT = 1.0 / 100.0  # 100 Hz position loop

# =============================================================================
# Attitude Rate PID Gains (inner loop)
# =============================================================================

PID_ROLL_RATE_KP = 200.0
PID_ROLL_RATE_KI = 500.0
PID_ROLL_RATE_KD = 0.0
PID_ROLL_RATE_KFF = 0.0
PID_ROLL_RATE_INTEGRATION_LIMIT = 33.3

PID_PITCH_RATE_KP = 200.0
PID_PITCH_RATE_KI = 500.0
PID_PITCH_RATE_KD = 0.0
PID_PITCH_RATE_KFF = 0.0
PID_PITCH_RATE_INTEGRATION_LIMIT = 33.3

PID_YAW_RATE_KP = 30.0
PID_YAW_RATE_KI = 16.7
PID_YAW_RATE_KD = 0.0
PID_YAW_RATE_KFF = 0.0
PID_YAW_RATE_INTEGRATION_LIMIT = 166.7

# =============================================================================
# Attitude PID Gains (outer loop)
# =============================================================================

PID_ROLL_KP = 12.0
PID_ROLL_KI = 3.0
PID_ROLL_KD = 0.0
PID_ROLL_KFF = 0.0
PID_ROLL_INTEGRATION_LIMIT = 20.0

PID_PITCH_KP = 12.0
PID_PITCH_KI = 3.0
PID_PITCH_KD = 0.0
PID_PITCH_KFF = 0.0
PID_PITCH_INTEGRATION_LIMIT = 20.0

PID_YAW_KP = 3.0
PID_YAW_KI = 1.0
PID_YAW_KD = 0.35
PID_YAW_KFF = 0.0
PID_YAW_INTEGRATION_LIMIT = 360.0

# =============================================================================
# Position PID Gains
# =============================================================================

PID_POS_X_KP = 5.0
PID_POS_X_KI = 0.1
PID_POS_X_KD = 0.0
PID_POS_X_KFF = 0.0

PID_POS_Y_KP = 5.0
PID_POS_Y_KI = 0.1
PID_POS_Y_KD = 0.0
PID_POS_Y_KFF = 0.0

PID_POS_Z_KP = 4.0
PID_POS_Z_KI = 0.5
PID_POS_Z_KD = 0.0
PID_POS_Z_KFF = 0.0

# Position velocity limits (m/s)
PID_POS_VEL_X_MAX = 1.0
PID_POS_VEL_Y_MAX = 1.0
PID_POS_VEL_Z_MAX = 1.0

# =============================================================================
# Velocity PID Gains
# =============================================================================

PID_VEL_X_KP = 100.0
PID_VEL_X_KI = 3.0
PID_VEL_X_KD = 0.0
PID_VEL_X_KFF = 0.0

PID_VEL_Y_KP = 100.0
PID_VEL_Y_KI = 3.0
PID_VEL_Y_KD = 0.0
PID_VEL_Y_KFF = 0.0

PID_VEL_Z_KP = 75.0
PID_VEL_Z_KI = 15.0
PID_VEL_Z_KD = 2.0
PID_VEL_Z_KFF = 0.0

# Velocity limits for attitude output
PID_VEL_ROLL_MAX = 20.0   # degrees
PID_VEL_PITCH_MAX = 20.0  # degrees

# Thrust base and minimum (in PWM scale 0-65535)
PID_VEL_THRUST_BASE = 30000.0
PID_VEL_THRUST_MIN = 20000.0
