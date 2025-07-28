import numpy as np

from math import tan, radians

DEFAULT_SETPOINT = (0.0, 0.0)

PATH_MAPPING = {"red": "circle", "green": "line", "blue": "infinity"}

# PINS
# connect in clockwise order
SERVO1_PIN = 17
SERVO2_PIN = 27
SERVO3_PIN = 22

# SERVO CONSTANTS
MIN_PULSEWIDTH = 500  # us
MAX_PULSEWIDTH = 2500
MIN_ANGLE = radians(-90)
MAX_ANGLE = radians(90)

SERVO1_OFFSET = radians(-45)
SERVO2_OFFSET = radians(-45)
SERVO3_OFFSET = radians(-48)

# ROBOT DIMENSIONS
R = 76.2  # mm
H = 85.0
X = 60.0
L1 = 60.0
L2 = 75.0
L3 = 91.2

# PID PARAMS
SAMPLE_TIME = 0.02  # s
ERROR_THRESHOLD = 5.0  # cv magnitude units
NORMAL_Z = 4.0

TILT_MAX = 32  # deg
MAX_XY = NORMAL_Z * tan(radians(TILT_MAX))

ALPHA = 0.55  # EMA low-pass filter constant (0.0-1.0)
MAX_INTEGRAL = 0.23  # max integral value to prevent windup
# DEGREE_DEADBAND = 0.5  # deg

KPX = 0.00389
KDX = 0.001937
KIX = 0.0000097

# KPX = 0.00395
# KDX = 0.00178
# KDX = 0.00182
# KIX = 0.00002

KPY = KPX
KDY = KDX
KIY = KIX

# MPU MADGEWICK PARAMS
NORMAL_CORRECTION_GAIN = 0.2
BETA = 0.1

# CV PARAMS
OUTPUT_SIZE = (320, 240)
FRAME_CENTRE = (OUTPUT_SIZE[0] // 2, OUTPUT_SIZE[1] // 2)

CIRCULARITY_THRESHOLD = 0.35
AREA_THRESHOLD = 100.0
KERNEL = np.ones((3, 3), np.uint8)
