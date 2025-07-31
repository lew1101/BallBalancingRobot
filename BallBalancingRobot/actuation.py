import pigpio  # type: ignore

from .constants import *


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
