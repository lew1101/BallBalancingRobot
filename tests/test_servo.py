from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

def test_servo():
    # Use pigpio as backend
    factory = PiGPIOFactory()

    # Create servo object on GPIO18 (BCM numbering)
    # Adjust pulse widths as needed
    servo1 = AngularServo(17, 
                        pin_factory=factory, 
                        min_pulse_width=0.5/1000, 
                        max_pulse_width=2.5/1000,
                        min_angle=90,
                        max_angle=-90,)
    servo2 = AngularServo(27, 
                        pin_factory=factory, 
                        min_pulse_width=0.5/1000, 
                        max_pulse_width=2.5/1000,
                        min_angle=90,
                        max_angle=-90,)
    servo3 = AngularServo(22, 
                        pin_factory=factory, 
                        min_pulse_width=0.5/1000, 
                        max_pulse_width=2.5/1000,
                        min_angle=90,
                        max_angle=-90,)

    print("Servo test: sweeping from 0° to 180°")

    try:
        while True:
            servo1.angle = -46 + 45
            servo2.angle = -46 + 45
            servo3.angle = -48 + 45

    except KeyboardInterrupt:
        print("Test stopped.")
