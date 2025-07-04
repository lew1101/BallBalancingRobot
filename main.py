import cv2
import numpy as np

import platform
from gpiozero import Servo  # type: ignore
# Use mock GPIO if not on Raspberry Pi (i.e. not ARM architecture)
if not platform.machine().startswith("arm"):
    from gpiozero import Device  # type: ignore
    from gpiozero.pins.mock import MockFactory  # type: ignore

    Device.pin_factory = MockFactory()

from simple_pid import PID

from time import time
from argparse import ArgumentParser

from kinematics import solveAngles
from vision import getBallPos

parser = ArgumentParser(description="Ball-balancing Robot")
parser.add_argument('--setpoint',
                    type=float,
                    nargs=2,
                    metavar=('setX', 'setY'),
                    help='Setpoint for ball position as two floats (x y)',
                    default=(0.0, 0.0))

# CONSTANTS
H = 0
X = 0
L1 = 0
L2 = 0
L3 = 0

NORMAL_Z = 4.0

KPX = 1.0
KIX = 0.1
KDX = 0.05

KPY = 1.0
KIY = 0.1
KDY = 0.05

LOWER_HSV = np.array([40, 70, 70])
UPPER_HSV = np.array([80, 255, 255])


def main():
    args = parser.parse_args()

    setPoint = (args.setX, args.setY)

    cap = cv2.VideoCapture(0)

    Servo1 = Servo()
    Servo2 = Servo()
    Servo3 = Servo()

    pidX = PID(Kp=KDX, Ki=KIX, Kd=KDX, sample_time=0)
    pidY = PID(Kp=KDY, Ki=KIY, Kd=KDY, sample_time=0)

    pidX.setpoint, pidY.setpoint = setPoint

    lastTime = time()

    try:
        while True:
            now = time()
            dt = now - lastTime
            lastTime = now

            ballX, ballY = getBallPos(cap, LOWER_HSV, UPPER_HSV) or setPoint

            planeNormal = (-pidX(ballX, dt), -pidY(ballY, dt), NORMAL_Z)

            angle1, angle2, angle3 = solveAngles(planeNormal, H, X, L1, L2, L3)

    except Exception as e:
        parser.error(str(e))

    finally:
        cap.release()


if __name__ == "__main__":
    main()
