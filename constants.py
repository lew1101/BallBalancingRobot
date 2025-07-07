import numpy as np

from math import tan

# PINS
SERVO1_PIN = 10  # dummy, need changing
SERVO2_PIN = 11
SERVO3_PIN = 12

# SERVO CONSTANTS
MIN_PULSEWIDTH = 500  # us
MAX_PULSEWIDTH = 2500

# ROBOT DIMENSIONS
R = 76.2  # mm
H = 120.0
X = 60.0
L1 = 75.0
L2 = 75.0
L3 = 86.2

# PID PARAMS
SAMPLE_TIME = 0.02  # s
ERROR_THRESHOLD = 3.0  # mm
NORMAL_Z = 1.0

TILT_MAX = 0.35  # rad
MAX_XY = NORMAL_Z / tan(TILT_MAX)

KPX = 1.0
KIX = 0.1
KDX = 0.05

KPY = 1.0
KIY = 0.1
KDY = 0.05

# CV MASK BOUNDS
LOWER_HSV = np.array([40, 70, 70])
UPPER_HSV = np.array([80, 255, 255])
