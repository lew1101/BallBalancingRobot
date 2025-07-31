#!/usr/bin/env python3

import cv2  # type : ignore
import pigpio  # type: ignore

from picamera2 import Picamera2  # type: ignore
from os import getenv
from argparse import ArgumentParser

# import csv
from time import monotonic, sleep
from math import hypot, fabs, atan2, degrees

from .constants import *
from .path import pathFactory
from .kinematics import solveAngles
from .vision import getBallPos, cvToRobotCoords, robotToCvCoords, COLOR_BGR
from .actuation import setArmPositions
from .control import PIDController
from .imu import createNormalEstimator


def parseArgs(argv=None):
    parser = ArgumentParser(description="Ball-balancing Robot")
    parser.add_argument(
        "--setpoint",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        help="Setpoint for ball position as two floats (x, y)",
        default=(0.0, 0.0),
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug mode",
        default=getenv("DEBUG"),
    )
    parser.add_argument(
        "--no-imu",
        action="store_true",
        help="Disable IMU",
        default=False,
    )

    return vars(parser.parse_args(argv))


def main(argv=None) -> None:
    kwargs = parseArgs(argv)
    

    DEBUG = kwargs.get("debug", False)
    IMU_ENABLED = not kwargs.get("no_imu", False)
    
    # logfile = open("pid_log.csv", "w", newline='')
    # logger = csv.writer(logfile)
    # logger.writerow(["time", "ballX", "ballY", "pid_x", "pid_y", "angle1", "angle2", "angle3"])

    # Init PIGPIO
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Failed to connect to the pigpio daemon. Is it running?\n"
                           "To start it, run:\n"
                           "    sudo pigpiod\n")

    try:
        setArmPositions(pi, 0, 0, 0)  # reset servos to 0 position

        if DEBUG:
            print("Debug mode enabled"
                  "Starting CV window thread")
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

        if IMU_ENABLED:
            print("IMU enabled, creating normal estimator")
            getNextNormal = createNormalEstimator(pi, beta=BETA)  # closure

        path = pathFactory("setpoint", kwargs.get("setpoint", DEFAULT_SETPOINT))
        pathiter = iter(path)

        setpoint = setX, setY = next(pathiter)
        print(f"Intial setpoint: ({setX}, {setY})")

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
            color, cv_centre, radius = getBallPos(frame, debug=DEBUG)

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

            if IMU_ENABLED:
                # correct normal based on IMU data
                actualNormal = getNextNormal(dt)
                normalX, normalY, _ = actualNormal * NORMAL_Z / actualNormal[2]
                normalCmdX = pidXVal + NORMAL_CORRECTION_GAIN * (normalX - pidXVal)
                normalCmdY = pidYVal + NORMAL_CORRECTION_GAIN * (normalY - pidYVal)
            else:
                normalCmdX = pidXVal
                normalCmdY = pidYVal
                
            
            # Check if clamping is necessary
            pidMag = hypot(pidXVal, pidYVal)    
            if pidMag > MAX_XY:
                # clamp tilt by clamping magnitude of xy vector
                correctionFactor = MAX_XY / pidMag
                normalCmdX = pidXVal * correctionFactor
                normalCmdY = pidYVal * correctionFactor
                print("Tilt clamped to max XY")

            # Check if setpoint needs to be updated
            errorMag = hypot(errorX, errorY)
            if errorMag < ERROR_THRESHOLD:
                # euclidean distance to setpoint is less than threshold
                # get next setpoint from path, with default of current point if fails.
                setpoint = setX, setY = next(pathiter, setpoint)
                pidX.updateSetpoint(setX)
                pidY.updateSetpoint(setY)
                print(f"Setpoint updated to: ({setX}, {setY})")

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


if __name__ == "__main__":
    main()
