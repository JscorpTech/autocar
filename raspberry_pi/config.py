import logging
import math
import os

# logging
LOG_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
LOG_LEVEL = logging.DEBUG

# serial
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# car dimensions
CAR_LENGTH = 2.0        # length, metres
CAR_WIDTH = 1.0         # width, metres
WHEEL_DIAMETER = 0.25
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
WHEEL_BASE = 1.8        # distance between front and rear axles
PULSES_PER_REV = 4

# steering (Ackermann) - BTS7960 steering motor control
# steer_angle: -30..+30 degrees -> PWM changes proportionally
MAX_STEER_ANGLE = 30.0  # maximum steering angle, degrees
MIN_TURN_RADIUS = WHEEL_BASE / math.tan(math.radians(MAX_STEER_ANGLE))
# 1.8 / tan(30°) ≈ 3.12m

# map
CELL_SIZE = 1.0

# motor (PWM 0-255, BTS7960)
BASE_SPEED = 150
MIN_SPEED = 80
MAX_SPEED = 220
TURN_SPEED = 100        # speed during turns (low)

# PID - straight driving (output: steering angle correction)
PID_KP = 1.5
PID_KI = 0.03
PID_KD = 0.4

# PID - turning (output: steering angle)
TURN_PID_KP = 1.0
TURN_PID_KI = 0.01
TURN_PID_KD = 0.3

# navigation tolerances
HEADING_TOLERANCE = 5.0     # degrees
DISTANCE_TOLERANCE = 0.1    # metres
WAYPOINT_REACHED = 0.15     # metres

# safety
OBSTACLE_DISTANCE = 0.5     # metres
COMMAND_RATE_HZ = 10

# headings (odometry-based, north = 0°)
NORTH = 0.0
EAST = 90.0
SOUTH = 180.0
WEST = 270.0

DIRECTION_MAP = {
    (-1, 0): NORTH,
    (1, 0): SOUTH,
    (0, 1): EAST,
    (0, -1): WEST,
}
