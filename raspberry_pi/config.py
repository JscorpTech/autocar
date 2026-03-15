import math

# serial
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# mashina gabariti
CAR_LENGTH = 2.0        # uzunligi, metr
CAR_WIDTH = 1.0         # eni, metr
WHEEL_DIAMETER = 0.25
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
WHEEL_BASE = 1.8        # old va orqa o'q orasidagi masofa
PULSES_PER_REV = 4

# rul (Ackermann) - BTS7960 rul motori boshqaruvi
# steer_angle: -30..+30 daraja -> PWM proportsional ravishda o'zgaradi
MAX_STEER_ANGLE = 30.0  # maksimal burilish burchagi, daraja
MIN_TURN_RADIUS = WHEEL_BASE / math.tan(math.radians(MAX_STEER_ANGLE))
# 1.8 / tan(30°) ≈ 3.12m

# xarita
CELL_SIZE = 1.0

# motor (PWM 0-255, BTS7960)
BASE_SPEED = 150
MIN_SPEED = 80
MAX_SPEED = 220
TURN_SPEED = 100        # burilish paytidagi tezlik (past)

# PID - to'g'ri yurish (chiqish: rul burchagi tuzatish)
PID_KP = 1.5
PID_KI = 0.03
PID_KD = 0.4

# PID - burilish (chiqish: rul burchagi)
TURN_PID_KP = 1.0
TURN_PID_KI = 0.01
TURN_PID_KD = 0.3

# navigatsiya toleranslari
HEADING_TOLERANCE = 5.0     # daraja
DISTANCE_TOLERANCE = 0.1    # metr
WAYPOINT_REACHED = 0.15     # metr

# xavfsizlik
OBSTACLE_DISTANCE = 0.5     # metr
COMMAND_RATE_HZ = 10

# yo'nalishlar (odometriya asosida, shimol = 0°)
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
