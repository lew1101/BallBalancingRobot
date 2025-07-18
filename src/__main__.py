import cv2

from time import monotonic, sleep
from math import hypot, tan, degrees
from simple_pid import PID
from gpiozero import Device, AngularServo  # type: ignore

from .constants import *
from .path import pathFactory
from .kinematics import solveAngles
from .vision import getBallPos, COLOR_BGR


def main(args):
    DEBUG = getattr(args, "debug", False)

    if DEBUG:
        from gpiozero.pins.mock import MockFactory, MockPWMPin  # type: ignore
        Device.pin_factory = MockFactory(pin_class=MockPWMPin)
    else:
        from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore
        Device.pin_factory = PiGPIOFactory()

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

        path = pathFactory("setpoint", getattr(args, "setPoint", DEFAULT_SETPOINT))

        pidX.setpoint, pidY.setpoint = path.initialPoint

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
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if cv_centre is None:
                continue

            height, width = frame.shape[:2]

            # change centre to the middle of image
            cvX, cvY = cv_centre
            ballX = cvX - width // 2
            ballY = height // 2 - cvY  # flip vertically

            # calculate x and y component of plane normal
            commandX = -pidX(ballX, dt)  # type: ignore
            commandY = -pidY(ballY, dt)  # type: ignore

            commandMag = hypot(commandX, commandY)

            # don't adjust if euclidean distance from setpoint does not exceed threshold
            # reduce jitter
            if commandMag < ERROR_THRESHOLD:
                nextSetpoint = path.next()
                if nextSetpoint is not None:
                    pidX.setpoint, pidY.setpoint = nextSetpoint
                continue

            # clamp tilt by clamping magnitude of xy vector
            elif commandMag > MAX_XY:
                correctionFactor = MAX_XY / commandMag
                commandX *= correctionFactor
                commandY *= correctionFactor

            print(
                f"x: {commandX:.1f}, y: {commandY:.1f}, tilt: {degrees(tan(NORMAL_Z / commandMag))}"
            )

            planeNormal = (commandX, commandY, NORMAL_Z)

            # set servo angles
            try:
                angle1, angle2, angle3 = solveAngles(planeNormal, H, X, L1, L2, L3)
                Servo1.angle = angle1
                Servo2.angle = angle2
                Servo3.angle = angle3
            except ValueError:
                continue

            # sleep until next sample time
            elapsed = monotonic() - start
            if (sleep_time := SAMPLE_TIME - elapsed) > 0:
                sleep(sleep_time)

    finally:
        cap.release()
        cv2.destroyAllWindows()
