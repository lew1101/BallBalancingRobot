import numpy as np

from math import tan, radians

DEFAULT_SETPOINT = (0.0, 0.0)

PATH_MAPPING = {"red": "circle", "green": "line", "blue": "infinity"}

# PINS
# connect in clockwise order
SERVO1_PIN = 17
SERVO2_PIN = 27
SERVO3_PIN = 22

SERVO1_OFFSET = -46
SERVO2_OFFSET = -46
SERVO3_OFFSET = -48

# SERVO CONSTANTS
MIN_PULSEWIDTH = 0.5/1000  # s
MAX_PULSEWIDTH = 2.5/1000
MIN_ANGLE = 90
MAX_ANGLE = -90

# ROBOT DIMENSIONS
R = 76.2  # mm
H = 100.0
X = 60.0
L1 = 60.0
L2 = 75.0
L3 = 91.2

# PID PARAMS
SAMPLE_TIME = 0.03  # s
ERROR_THRESHOLD = 0.1
NORMAL_Z = 4.0

TILT_MAX = 20  # deg
MAX_XY = NORMAL_Z * tan(radians(TILT_MAX))

KPX = 0.01
KIX = 0.0
KDX = 0.002

KPY = 0.01
KIY = 0.0
KDY = 0.002

# CV PARAMS
OUTPUT_SIZE = (320, 320)  # low res, square
FRAME_CENTRE = (OUTPUT_SIZE[0] // 2, OUTPUT_SIZE[1] // 2)

CIRCULARITY_THRESHOLD = 0.35
AREA_THRESHOLD = 200.0
KERNEL = np.ones((3, 3), np.uint8)
