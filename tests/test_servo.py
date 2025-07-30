import pigpio  # type: ignore

from math import radians

from src.constants import *
from src.actuation import setArmPositions


def test_servo():
    # Use pigpio as backend
    print("Servo test: Set to 45 deg")

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Failed to connect to pigpio daemon. Is it running?")

    try:
        angle = radians(45)

        setArmPositions(pi, angle, angle, angle)

    except KeyboardInterrupt:
        print("Test stopped.")

    finally:
        pi.close()
