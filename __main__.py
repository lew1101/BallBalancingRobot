import os
import cv2

from gpiozero import Device, AngularServo  # type: ignore
if os.getenv("DEBUG"):
    from gpiozero.pins.mock import MockFactory  # type: ignore
    Device.pin_factory = MockFactory()
else:
    from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore
    Device.pin_factory = PiGPIOFactory()

from simple_pid import PID

from time import monotonic, sleep
from math import hypot
from argparse import ArgumentParser

from .constants import *
from .kinematics import solveAngles
from .vision import getBallPos

parser = ArgumentParser(description="Ball-balancing Robot")
parser.add_argument('--setpoint',
                    type=float,
                    nargs=2,
                    metavar=('setX', 'setY'),
                    help='Setpoint for ball position as two floats (x y)',
                    default=(0.0, 0.0))


def main():
    try:
        args = parser.parse_args()

        setPoint = (args.setX, args.setY)

        cap = cv2.VideoCapture(0)

        Servo1 = AngularServo(SERVO1_PIN,
                              initial_angle=0,
                              min_pulse_width=MIN_PULSEWIDTH,
                              max_pulse_width=MAX_PULSEWIDTH)
        Servo2 = AngularServo(SERVO2_PIN,
                              initial_angle=0,
                              min_pulse_width=MIN_PULSEWIDTH,
                              max_pulse_width=MAX_PULSEWIDTH)
        Servo3 = AngularServo(SERVO3_PIN,
                              initial_angle=0,
                              min_pulse_width=MIN_PULSEWIDTH,
                              max_pulse_width=MAX_PULSEWIDTH)

        pidX = PID(Kp=KDX, Ki=KIX, Kd=KDX, sample_time=0)
        pidY = PID(Kp=KDY, Ki=KIY, Kd=KDY, sample_time=0)

        pidX.setpoint, pidY.setpoint = setPoint

        lastTime = monotonic()

        while True:
            start = monotonic()
            dt = start - lastTime
            lastTime = start

            ballPos = getBallPos(cap, LOWER_HSV, UPPER_HSV)

            if ballPos is None:
                continue

            commandX = -pidX(ballPos[0], dt)
            commandY = -pidY(ballPos[1], dt)

            commandMag = hypot(commandX, commandY)

            # don't adjust if euclidean distance from setpoint does not exceed threshold
            # reduce jitter
            if commandMag < ERROR_THRESHOLD:
                continue
            elif commandMag > MAX_XY:
                correctionFactor = MAX_XY / commandMag
                commandX *= correctionFactor
                commandY *= correctionFactor

            planeNormal = (commandX, commandY, NORMAL_Z)

            # set servo angles
            Servo1.angle, Servo2.angle, Servo3.angle = solveAngles(planeNormal, H, X, L1, L2, L3)

            elapsed = monotonic() - start

            if (sleep_time := SAMPLE_TIME - elapsed) > 0:
                sleep(sleep_time)

    finally:
        cap.release()


if __name__ == "__main__":
    main()
