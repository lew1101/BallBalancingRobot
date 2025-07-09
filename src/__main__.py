import cv2

from time import monotonic, sleep
from math import hypot
from argparse import ArgumentParser
from simple_pid import PID
from gpiozero import Device, AngularServo  # type: ignore

from os import getenv

DEBUG = getenv("DEBUG")

print(bool(DEBUG))

if DEBUG:
    from gpiozero.pins.mock import MockFactory  # type: ignore
    Device.pin_factory = MockFactory()
else:
    from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore
    Device.pin_factory = PiGPIOFactory()

from .constants import *
from .path import Path, createSetPoint
from .kinematics import solveAngles
from .vision import getBallPos, COLOR_BGR

parser = ArgumentParser(description="Ball-balancing Robot")
parser.add_argument('--setpoint',
                    type=float,
                    nargs=2,
                    metavar='setPoint',
                    help='Setpoint for ball position as two floats (x y)',
                    default=(0.0, 0.0))


def main(**kwargs):
    try:
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

        if "path" in kwargs:
            path = Path(kwargs.path, loop=kwargs.get("loop"))
        else:
            path = createSetPoint(kwargs.get("setPoint", DEFAULT_SETPOINT))

        pidX.setpoint, pidY.setpoint = path.initalPoint

        lastTime = monotonic()

        while True:
            start = monotonic()
            dt = start - lastTime
            lastTime = start

            ret, frame = cap.read()
            if not ret:
                break

            color, cv_centre, radius = getBallPos(frame, debug=DEBUG)

            if DEBUG:
                if color and cv_centre:
                    cv2.circle(frame, cv_centre, radius, COLOR_BGR[color], 2)
                    cv2.putText(frame, color.upper(), (cv_centre[0] - 20, cv_centre[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_BGR[color], 2)
                cv2.imshow("Camera", frame)

            if cv_centre is None:
                continue

            height, width = frame.shape[:2]

            # change centre to the middle of image
            cvX, cvY = cv_centre
            ballX = cvX - width // 2
            ballY = height // 2 - cvY  # flip vertically

            # calculate x and y component of plane normal
            commandX = -pidX(ballX[0], dt)
            commandY = -pidY(ballY[1], dt)

            commandMag = hypot(commandX, commandY)

            # don't adjust if euclidean distance from setpoint does not exceed threshold
            # reduce jitter
            if commandMag < ERROR_THRESHOLD:
                if path.hasNext():
                    pidX.setpoint, pidY.setpoint = path.next()
                continue

            # clamp tilt by clamping magnitude of xy vector
            elif commandMag > MAX_XY:
                correctionFactor = MAX_XY / commandMag
                commandX *= correctionFactor
                commandY *= correctionFactor

            planeNormal = (commandX, commandY, NORMAL_Z)

            # set servo angles
            Servo1.angle, Servo2.angle, Servo3.angle = solveAngles(planeNormal, H, X, L1, L2, L3)

            # sleep until next sample time
            elapsed = monotonic() - start
            if (sleep_time := SAMPLE_TIME - elapsed) > 0:
                sleep(sleep_time)

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    kwargs = parser.parse_args()
    main(**kwargs)
