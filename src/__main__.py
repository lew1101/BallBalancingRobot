import cv2
import pigpio  # type: ignore
from picamera2 import Picamera2  # type: ignore

# import csv
from time import monotonic, sleep
from math import hypot, fabs, atan2, degrees

from .constants import *
from .path import pathFactory
from .kinematics import solveAngles
from .vision import getBallPos, cvToRobotCoords, robotToCvCoords, COLOR_BGR
from .control import PIDController
from .imu import createNormalEstimator


def angleToPulsewidth(angle: float,
                      *,
                      min_angle=MIN_ANGLE,
                      max_angle=MAX_ANGLE,
                      min_pulse=MIN_PULSEWIDTH,
                      max_pulse=MAX_PULSEWIDTH,
                      reverse=False) -> int:

    angle = max(min(angle, max_angle), min_angle)

    # Reverse angle if needed
    if reverse:
        angle = max_angle - (angle - min_angle)

    # Linear interpolation
    scale = (angle - min_angle) / (max_angle - min_angle)
    pulse = min_pulse + scale * (max_pulse - min_pulse)

    return int(pulse)


def setArmPositions(pi: pigpio.pi, angle1: float, angle2: float, angle3: float) -> None:
    pi.set_servo_pulsewidth(SERVO1_PIN, angleToPulsewidth(angle1 + SERVO1_OFFSET, reverse=True))
    pi.set_servo_pulsewidth(SERVO2_PIN, angleToPulsewidth(angle2 + SERVO2_OFFSET, reverse=True))
    pi.set_servo_pulsewidth(SERVO3_PIN, angleToPulsewidth(angle3 + SERVO3_OFFSET, reverse=True))


def main(args):
    DEBUG = getattr(args, "debug", False)

    # logfile = open("pid_log.csv", "w", newline='')
    # logger = csv.writer(logfile)
    # logger.writerow(["time", "ballX", "ballY", "pid_x", "pid_y", "angle1", "angle2", "angle3"])

    # Init PIGPIO
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Failed to connect to pigpio daemon. Is it running?")

    try:
        setArmPositions(pi, 0, 0, 0)  # reset servos to 0 position

        if DEBUG:
            print("Starting CV window thread")
            cv2.startWindowThread()

        # Init PICAMERA
        print("Starting Picamera2")
        picam2 = Picamera2()
        sensor_w, sensor_h = picam2.sensor_resolution
        config = picam2.create_preview_configuration(main={
            "size": (sensor_w // 4, sensor_h // 4),
            'format': 'RGB888'
        },)
        picam2.configure(config)
        picam2.start()

        # =====

        path = pathFactory("setpoint", getattr(args, "setpoint", DEFAULT_SETPOINT))
        pathiter = iter(path)

        setpoint = setX, setY = next(pathiter)
        print(f"Intial setpoint: ({setX}, {setY})")

        getNextNormal = createNormalEstimator(pi, SAMPLE_TIME, BETA)  # closure

        pidX = PIDController(kp=KPX,
                             ki=KIX,
                             kd=KDX,
                             setpoint=setX,
                             alpha=ALPHA,
                             maxIntegral=MAX_INTEGRAL)
        pidY = PIDController(kp=KPY,
                             ki=KIY,
                             kd=KDY,
                             setpoint=setY,
                             alpha=ALPHA,
                             maxIntegral=MAX_INTEGRAL)

        lastTime = monotonic()

        while True:
            start = monotonic()
            dt = start - lastTime
            lastTime = start

            # setArmPositions(pi, 0, 0, 0)
            # continue

            request = picam2.capture_request()
            frame = request.make_array("main")
            request.release()

            if frame is None:
                break
            # Downsample frame to reduce processing time
            square_crop = frame[::2, ::2]  # downsample by 2x
            frame = cv2.resize(square_crop, OUTPUT_SIZE, interpolation=cv2.INTER_NEAREST)

            # get ball position
            color, cv_centre, radius = getBallPos(frame, debug=False)

            if DEBUG:
                # draw current setpoint
                cvSetX, cvSetY = robotToCvCoords(setpoint)
                cv2.circle(frame, (int(cvSetX), int(cvSetY)),
                           radius=4,
                           color=(0, 0, 255),
                           thickness=-1)

            if cv_centre is None:  # no ball detected
                setArmPositions(pi, 0, 0, 0)
                if DEBUG:
                    cv2.imshow("Preview", frame)
                continue  # don't sleep, get next frame immediately

            # convert cv coordinates to robot coordinates
            ballX, ballY = cvToRobotCoords(cv_centre)

            if DEBUG:
                # draw ball position
                cx, cy = cv_centre
                cv2.circle(frame, (int(cx), int(cy)), int(radius), COLOR_BGR[color], 2)
                cv2.putText(frame, f"({ballX:.2f}, {ballY:.2f})", (int(cx) - 20, int(cy) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_BGR[color], 2)
                cv2.imshow("Preview", frame)

            # calculate x and y component of plane normal
            pidXVal, errorX = pidX(ballX, dt)
            pidYVal, errorY = pidY(ballY, dt)

            pidMag = hypot(pidXVal, pidYVal)
            errorMag = hypot(errorX, errorY)

            #
            actualNormal = getNextNormal(dt)
            normalX, normalY, _ = actualNormal * NORMAL_Z / actualNormal[2]
            normalCmdX = pidXVal + NORMAL_CORRECTION_GAIN * (normalX - pidXVal)
            normalCmdY = pidYVal + NORMAL_CORRECTION_GAIN * (normalY - pidYVal)

            if errorMag < ERROR_THRESHOLD:
                # euclidean distance to setpoint is less than threshold
                # get next setpoint from path, with default of current point if fails.
                setpoint = setX, setY = next(pathiter, setpoint)
                pidX.updateSetpoint(setX)
                pidY.updateSetpoint(setY)
                print(f"Setpoint updated to: ({setX}, {setY})")

            elif pidMag > MAX_XY:
                # clamp tilt by clamping magnitude of xy vector
                correctionFactor = MAX_XY / pidMag
                normalCmdX = pidXVal * correctionFactor
                normalCmdY = pidYVal * correctionFactor
                print("Tilt clamped to max XY")

            # print(f"tilt: {degrees(atan2(hypot(normalCmdX, normalCmdY), NORMAL_Z)):.1f}")
            planeNormal = (normalCmdX, normalCmdY, NORMAL_Z)
            angles = solveAngles(planeNormal, H, X, L1, L2, L3)
            setArmPositions(pi, *angles)

            # logger.writerow([start, ballX, ballY, pidXVal, pidYVal, *angles])

            # sleep until next sample time
            elapsed = monotonic() - start
            sleep_time = SAMPLE_TIME - elapsed
            if sleep_time > 0:
                # print(f"Sleeping for {sleep_time:.4f}s")
                sleep(sleep_time)
            else:
                # print(f"Not sleeping, sleep_time = {sleep_time:.4f}s")
                pass
    except KeyboardInterrupt:
        print("KeyboardInterrupt, stopping...")

    finally:
        setArmPositions(pi, 0, 0, 0)

        if picam2:
            picam2.stop()

        pi.stop()

        # logfile.close()
        cv2.destroyAllWindows()
