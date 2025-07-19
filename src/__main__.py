import cv2
from picamera2 import Picamera2

from time import monotonic, sleep
from math import hypot, fabs, atan2, degrees
from simple_pid import PID
from gpiozero import Device, AngularServo  # type: ignore

import numpy as np

from .constants import *
from .path import pathFactory
from .kinematics import solveAngles
from .vision import getBallPos, COLOR_BGR


def main(args):
    DEBUG = getattr(args, "debug", False)

    from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore
    Device.pin_factory = PiGPIOFactory()
        
    try:
        # print("Starting Picamera2")
        picam2 = Picamera2()
        
        sensor_w, sensor_h = picam2.sensor_resolution
        side = min(sensor_w, sensor_h)
        
        # crop_x = (sensor_w - side) // 16
        # crop_y = (sensor_h - side) // 16

        config = picam2.create_preview_configuration(
            main={"size": (sensor_w // 4, sensor_h // 4), 'format': 'RGB888'},
        )
        picam2.configure(config)
        picam2.start()
       
        cv2.startWindowThread()

        Servo1 = AngularServo(SERVO1_PIN,
                              initial_angle=0,
                              min_pulse_width=MIN_PULSEWIDTH,
                              max_pulse_width=MAX_PULSEWIDTH,
                              min_angle=MIN_ANGLE,
                              max_angle=MAX_ANGLE)
        Servo2 = AngularServo(SERVO2_PIN,
                              initial_angle=0,
                              min_pulse_width=MIN_PULSEWIDTH,
                              max_pulse_width=MAX_PULSEWIDTH,
                              min_angle=MIN_ANGLE,
                              max_angle=MAX_ANGLE)
        Servo3 = AngularServo(SERVO3_PIN,
                              initial_angle=0,
                              min_pulse_width=MIN_PULSEWIDTH,
                              max_pulse_width=MAX_PULSEWIDTH,
                              min_angle=MIN_ANGLE,
                              max_angle=MAX_ANGLE)

        pidX = PID(Kp=KPX, Ki=KIX, Kd=KDX, sample_time=0)
        pidY = PID(Kp=KPY, Ki=KIY, Kd=KDY, sample_time=0)
        
        
        # path = pathFactory("circle", radius=MAX_XY, n=100)

        path = pathFactory("setpoint", getattr(args, "setPoint", DEFAULT_SETPOINT))
        pathiter = iter(path)

        setPoint = pidX.setpoint, pidY.setpoint = next(pathiter)

        lastTime = monotonic()

        while True:
            start = monotonic()
            dt = start - lastTime
            lastTime = start

            request = picam2.capture_request()
            frame = request.make_array("main")  
            request.release()
    
            if frame is None:
                break
           
            square_crop = frame[::2, ::2]  # downsample by 2x
            frame = cv2.resize(square_crop, OUTPUT_SIZE, interpolation=cv2.INTER_NEAREST)
            
            color, cv_centre, radius = getBallPos(frame, debug=False)
            
            if cv_centre is None:
                if DEBUG:
                    cv2.imshow("Preview", frame)
                continue # get next frame as fast as possible
            
            cx, cy = cv_centre

            ballX = cy - OUTPUT_SIZE[1] // 2
            ballY = OUTPUT_SIZE[0] // 2 - cx

            if DEBUG:
                cv2.circle(frame, (int(cx), int(cy)) , int(radius), COLOR_BGR[color], 2)
                cv2.putText(frame, f"({ballX:.2f}, {ballY:.2f})", (int(cx) - 20, int(cy) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_BGR[color], 2)
                cv2.imshow("Preview", frame)
                
            # calculate x and y component of plane normal
            commandX = pidX(ballX, dt)  # type: ignore
            commandY = pidY(ballY, dt)  # type: ignore
            
            # print(f"PID: {commandX}, {commandY}")
            # print(f"Ball: {ballX}, {ballY}")

            commandMag = hypot(commandX, commandY)

            # don't adjust if euclidean distance from setpoint does not exceed threshold
            # reduce jitter
            if commandMag < ERROR_THRESHOLD:
                nextSetpoint = next(pathiter, setPoint)
                if nextSetpoint is not None:
                    setPoint = pidX.setpoint, pidY.setpoint = nextSetpoint
                continue

            # clamp tilt by clamping magnitude of xy vector
            elif commandMag > MAX_XY:
                correctionFactor = MAX_XY / commandMag
                commandX *= correctionFactor
                commandY *= correctionFactor

            # print(
                # f"x: {commandX:.2f}, y: {commandY:.2f}, tilt: {degrees(atan2(hypot(commandX, commandY), NORMAL_Z)):.1f}"
            # )
            
            # print(f"Setpoint: {commandX:.2f}, {commandY:.2f}")

            planeNormal = (commandX, commandY, NORMAL_Z)

            # set servo angles (in radians)
            angle1, angle2, angle3 = solveAngles(planeNormal, H, X, L1, L2, L3)
            Servo1.angle = degrees(angle1) + SERVO1_OFFSET
            Servo2.angle = degrees(angle2) + SERVO2_OFFSET
            Servo3.angle = degrees(angle3) + SERVO3_OFFSET
            
            # print(f"Angles: {degrees(angle1)}, {degrees(angle2)}, {degrees(angle3)}")

            # sleep until next sample time
            elapsed = monotonic() - start
            sleep_time = SAMPLE_TIME - elapsed
            if sleep_time > 0:
                print(f"Sleeping for {sleep_time:.4f}s")
                sleep(sleep_time)
            else:
                print(f"Not sleeping, sleep_time = {sleep_time:.4f}s")

    finally:
        Servo1.angle = SERVO1_OFFSET
        Servo2.angle = SERVO2_OFFSET
        Servo3.angle = SERVO3_OFFSET
        
        cv2.destroyAllWindows()
        picam2.stop()
