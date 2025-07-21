import numpy as np

from math import tan, radians

DEFAULT_SETPOINT = (0.0, 0.0)

PATH_MAPPING = {"red": "circle", "green": "line", "blue": "infinity"}

# PINS
# connect in clockwise order
SERVO1_PIN = 17
SERVO2_PIN = 27
SERVO3_PIN = 22

SERVO1_OFFSET = -45
SERVO2_OFFSET = -46
SERVO3_OFFSET = -48

# SERVO CONSTANTS
MIN_PULSEWIDTH = 500  # us
MAX_PULSEWIDTH = 2500
MIN_ANGLE = -90
MAX_ANGLE = 90

# ROBOT DIMENSIONS
R = 76.2  # mm
H = 75.0
X = 60.0
L1 = 60.0
L2 = 75.0
L3 = 91.2

# PID PARAMS
SAMPLE_TIME = 0.02  # s
ERROR_THRESHOLD = 1.0
NORMAL_Z = 4.0

TILT_MAX = 32  # deg
MAX_XY = NORMAL_Z * tan(radians(TILT_MAX))

ALPHA = 0.45 # low-pass filter constant (0.0-1.0)

KPX = 0.00397
KDX = 0.00184
KIX = 0.00006

# KPX = 0.00395
# KDX = 0.0018
# KIX = 0.00003


KPY = KPX
KDY = KDX
KIY = KIX

# CV PARAMS
OUTPUT_SIZE = (320, 240) 
FRAME_CENTRE = (OUTPUT_SIZE[0] // 2, OUTPUT_SIZE[1] // 2)

CIRCULARITY_THRESHOLD = 0.35
AREA_THRESHOLD = 120.0
KERNEL = np.ones((3, 3), np.uint8)
