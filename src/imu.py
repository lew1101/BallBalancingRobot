import pigpio # type: ignore
import numpy as np

from ahrs.filters import Madgwick # type: ignore
from ahrs.common.orientation import q2mat # type: ignore

from .constants import *

MPU6050_ADDRESS = 0x68
MPU6050_DATA_START = 0x3B
MPU6050_DATA_BLOCK_SIZE = 14

ACCEL_SCALE = 16384.0  # scale factor for accelerometer data
GYRO_SCALE = 131.0  # scale factor for gyroscope data

def bytesToInt16(high: int, low: int) -> int:
    # Convert two bytes to a signed 16-bit integer
    value = (high << 8) | low
    return value if value < 32768 else value - 65536


def streamMPUData(pi: pigpio.pi):   
    """
    Generator that yields scaled accelerometer and gyroscope data from MPU6050.
    Returns: ((ax, ay, az), (gx, gy, gz))
    """

    handle = pi.i2c_open(1, MPU6050_ADDRESS)

    try:
        while True:
            count, block = pi.i2c_read_i2c_block_data(handle, MPU6050_DATA_START, MPU6050_DATA_BLOCK_SIZE)
            if count != MPU6050_DATA_BLOCK_SIZE:
                raise RuntimeError(f"Expected 14 bytes, got {count}")
            
            # Extract accelerometer and gyroscope data from the block
            ax = bytesToInt16(block[0], block[1]) / ACCEL_SCALE
            ay = bytesToInt16(block[2], block[3]) / ACCEL_SCALE
            az = bytesToInt16(block[4], block[5]) / ACCEL_SCALE
            
            gx = bytesToInt16(block[8], block[9]) / GYRO_SCALE
            gy = bytesToInt16(block[10], block[11]) / GYRO_SCALE
            gz = bytesToInt16(block[12], block[13]) / GYRO_SCALE
            
            yield (gx, gy, gz), (ax, ay, az)

    finally:
        pi.i2c_close(handle)
    

def streamMPUNormal(pi: pigpio.pi, sampleTime: int, beta: float = 0.1):
    mag = Madgwick(beta=beta, Dt=sampleTime)
    
    for gyro, acc in streamMPUData(pi):
        q = mag.updateIMU(gyr=np.deg2rad(gyro), acc=np.array(acc))
        R = q2mat(q)
        
        # plane normal vector is the z column of the rotation matrix
        normal = R[:, 2] 
        yield normal
