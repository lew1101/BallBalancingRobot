import numpy as np

from math import tan

DEFAULT_SETPOINT = (0.0, 0.0)

PATH_MAPPING = {"red": "circle", "green": "line", "blue": "infinity"}

# PINS
SERVO1_PIN = 17
SERVO2_PIN = 27
SERVO3_PIN = 22

# SERVO CONSTANTS
MIN_PULSEWIDTH = 0.001  # s
MAX_PULSEWIDTH = 0.002

# ROBOT DIMENSIONS
R = 76.2  # mm
H = 100.0
X = 60.0
L1 = 60.0
L2 = 75.0
L3 = 91.2

# PID PARAMS
SAMPLE_TIME = 0.02  # s
ERROR_THRESHOLD = 1.0
NORMAL_Z = 4.0

TILT_MAX = 0.35  # rad
MAX_XY = NORMAL_Z / tan(TILT_MAX)

KPX = 0.005
KIX = 0.0
KDX = 0.0

KPY = 0.005
KIY = 0.0
KDY = 0.0

# CV PARAMS
CIRCULARITY_THRESHOLD = 0.35
AREA_THRESHOLD = 400.0
KERNEL = np.ones((3, 3), np.uint8)
