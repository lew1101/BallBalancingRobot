import cv2
from time import sleep
from math import degrees 

from gpiozero import Device, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore
Device.pin_factory = PiGPIOFactory()
        
from time import sleep

from picamera2 import Picamera2

from src.kinematics import solveAngles
from src.vision import *
from src.constants import *

def test_vision():
    try:
        picam2 = Picamera2()
        
        Servo1 = AngularServo(17, 
                        min_pulse_width=0.5/1000, 
                        max_pulse_width=2.5/1000,
                        min_angle=90,
                        max_angle=-90,)
        Servo2 = AngularServo(27, 
                            min_pulse_width=0.5/1000, 
                            max_pulse_width=2.5/1000,
                            min_angle=90,
                            max_angle=-90,)
        Servo3 = AngularServo(22, 
                            min_pulse_width=0.5/1000, 
                            max_pulse_width=2.5/1000,
                            min_angle=90,
                            max_angle=-90,)
        
        planeNormal = (0.0, 0.0, NORMAL_Z)

        # set servo angles (in radians)
        angle1, angle2, angle3 = solveAngles(planeNormal, H, X, L1, L2, L3)
        Servo1.angle = degrees(angle1) + SERVO1_OFFSET
        Servo2.angle = degrees(angle2) + SERVO2_OFFSET
        Servo3.angle = degrees(angle3) + SERVO3_OFFSET

        
        sensor_w, sensor_h = picam2.sensor_resolution
        side = min(sensor_w, sensor_h)

        print(f"Sensor resolution: {sensor_w}x{sensor_h}, side: {side}")
        
        output_size = 320  # Desired output size (low res, square)

        config = picam2.create_preview_configuration(
            main={"size": (sensor_w // 4, sensor_h // 4), 'format': 'RGB888'},
        )
        picam2.configure(config)
        picam2.start()
       
        cv2.startWindowThread()
        
        crop_x = (sensor_w - side) // 8
        crop_y = (sensor_h - side) // 8

        while True:
            frame = picam2.capture_array()
           
            square_crop = frame[crop_y:crop_y+sensor_w // 4, crop_x:crop_x+sensor_h // 4]

            # Downscale to output size (e.g., 320x320)
            frame = cv2.resize(square_crop, (output_size, output_size))


            color, centre, radius = getBallPos(frame, debug=True)
            
            if color and centre:
                x = centre[1] - output_size // 2
                y = output_size // 2 - centre[0]
                
                cv2.circle(frame, centre, radius, COLOR_BGR[color], 2)
                cv2.putText(frame, f"{(x, y)}", (centre[0] - 20, centre[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_BGR[color], 2)

            cv2.imshow("Camera", frame)

            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        Servo1.angle = SERVO1_OFFSET
        Servo2.angle = SERVO2_OFFSET
        Servo3.angle = SERVO3_OFFSET
        
        picam2.stop()
        cv2.destroyAllWindows()
        
        
