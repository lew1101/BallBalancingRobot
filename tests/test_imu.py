import pigpio  # type: ignore

from time import monotonic, sleep

from src.imu import *
from src.constants import *


def test_imu():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Failed to connect to pigpio daemon. Is it running?")

    getNormal = createNormalEstimator(pi, beta=BETA)

    lastTime = monotonic()

    try:
        while True:
            start = monotonic()
            dt = start - lastTime
            lastTime = start

            normal = getNormal(dt)

            print(f"Normal vector: {normal}")

            elapsed = monotonic() - start
            if elapsed < SAMPLE_TIME:
                sleep(SAMPLE_TIME - elapsed)

    finally:
        pi.stop()
