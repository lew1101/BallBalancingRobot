import cv2
import pigpio
from picamera2 import Picamera2

import csv
from time import monotonic, sleep
from math import hypot, fabs, atan2, degrees

from .constants import *
from .path import pathFactory
from .kinematics import solveAngles
from .vision import getBallPos, COLOR_BGR
from .control import PIDController


def angleToPulsewidth(angle, *, min_angle=MIN_ANGLE, max_angle=MAX_ANGLE,
                        min_pulse=MIN_PULSEWIDTH, max_pulse=MAX_PULSEWIDTH, reverse=False):
    
    angle = max(min(angle, max_angle), min_angle)

    # Reverse angle if needed
    if reverse:
        angle = max_angle - (angle - min_angle)

    # Linear interpolation
    scale = (angle - min_angle) / (max_angle - min_angle)
    pulse = min_pulse + scale * (max_pulse - min_pulse)
    
    return int(pulse)

def setArmPositions(pi, angle1, angle2, angle3):
    pi.set_servo_pulsewidth(SERVO1_PIN, angleToPulsewidth(angle1 + SERVO1_OFFSET, reverse=True))
    pi.set_servo_pulsewidth(SERVO2_PIN, angleToPulsewidth(angle2 + SERVO2_OFFSET, reverse=True))
    pi.set_servo_pulsewidth(SERVO3_PIN, angleToPulsewidth(angle3 + SERVO3_OFFSET, reverse=True))


def main(args):
    DEBUG = getattr(args, "debug", False)
    
    # logfile = open("pid_log.csv", "w", newline='')
    # logger = csv.writer(logfile)
    # logger.writerow(["time", "ballX", "ballY", "pid_x", "pid_y", "angle1", "angle2", "angle3"])

    try:
        pi = pigpio.pi()
        
        if not pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon. Is it running?")    
        
        setArmPositions(pi, 0, 0, 0)  # reset servos to 0 position
        
        print("Starting Picamera2")
        picam2 = Picamera2()
        
        sensor_w, sensor_h = picam2.sensor_resolution
        side = min(sensor_w, sensor_h)

        config = picam2.create_preview_configuration(
            main={"size": (sensor_w // 4, sensor_h // 4), 'format': 'RGB888'},
        )
        picam2.configure(config)
        picam2.start()
       
        cv2.startWindowThread()
        # path = pathFactory("circle", radius=35, n=100)

        path = pathFactory("setpoint", getattr(args, "setpoint", DEFAULT_SETPOINT))
        pathiter = iter(path)
        
        setpoint = setX, setY = next(pathiter)
                
        print(setX, setY)
        pidX = PIDController(kp=KPX, ki=KIX, kd=KDX, setpoint=setX, alpha=ALPHA)
        pidY = PIDController(kp=KPY, ki=KIY, kd=KDY, setpoint=setY, alpha=ALPHA)

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
                setArmPositions(pi, 0, 0, 0)  # reset servos to 0 position

                if DEBUG:
                    cv2.imshow("Preview", frame)
                continue # get next frame as fast as possible
            
            cx, cy = cv_centre

            # convert cv coordinates to robot coordinates
            # cv2 coordinates: (x, y) = (col, row)
            # robot coordinates: (x, y) = (y, -x)
            # so we need to swap x and y, and negate x
            ballX = cy - OUTPUT_SIZE[1] // 2
            ballY = OUTPUT_SIZE[0] // 2 - cx

            if DEBUG:
                cv2.circle(frame, (int(cx), int(cy)) , int(radius), COLOR_BGR[color], 2)
                cv2.putText(frame, f"({ballX:.2f}, {ballY:.2f})", (int(cx) - 20, int(cy) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_BGR[color], 2)
                cv2.imshow("Preview", frame)
                
            # calculate x and y component of plane normal
            pidXVal, errorX = pidX(ballX, dt)  # type: ignore
            pidYVal, errorY = pidY(ballY, dt)  # type: ignore

            pidMag = hypot(pidXVal, pidYVal)
            
            errorMag = hypot(errorX, errorY)

            # don't adjust if euclidean distance from setpoint does not exceed threshold
            # reduce jitter
            if errorMag < ERROR_THRESHOLD:
                setpoint = setX, setY = next(pathiter, setpoint) # get next setpoint from path, with default of current point if fails. 
                
                pidX.updateSetpoint(setX)
                pidY.updateSetpoint(setY)
                
                print(f"Setpoint updated to: {setX}, {setY}")
                continue

            # clamp tilt by clamping magnitude of xy vector
            elif pidMag > MAX_XY:
                correctionFactor = MAX_XY / pidMag
                commandX = pidXVal * correctionFactor
                commandY = pidYVal * correctionFactor

                print("tilt clamped to max XY")
            else:
                commandX = pidXVal
                commandY = pidYVal
                
            # print(f"x: {commandX:.2f}, y: {commandY:.2f}, tilt: {degrees(atan2(hypot(commandX, commandY), NORMAL_Z)):.1f}")

            planeNormal = (commandX, commandY, NORMAL_Z)

            # set servo angles (in radians)
            angle1, angle2, angle3 = solveAngles(planeNormal, H, X, L1, L2, L3)
            setArmPositions(pi, angle1, angle2, angle3)
            
            # print(f"Angles: {angle1}, {angle2}, {angle3}")
            
            # logger.writerow([start, ballX, ballY, pidXVal, pidYVal, angle1, angle2, angle3])

            # sleep until next sample time
            elapsed = monotonic() - start
            sleep_time = SAMPLE_TIME - elapsed
            if sleep_time > 0:
                # print(f"Sleeping for {sleep_time:.4f}s")
                sleep(sleep_time)
            else:
                pass
                # print(f"Not sleeping, sleep_time = {sleep_time:.4f}s")

    finally:
        setArmPositions(pi, 0, 0, 0)  # reset servos to 0 position
        
        # logfile.close()
        cv2.destroyAllWindows()
        
        pi.stop()
        picam2.stop()
