import board
import busio
import adafruit_bno055

import gpio

IMU_FREQ = 100000

i2c = busio.I2C(
    scl=board.GP7,
    sda=board.GP6,
    frequency=IMU_FREQ
)

IMU = adafruit_bno055.BNO055_I2C(i2c)


def read_angle():
    try:
        euler = gpio.IMU.euler

        if euler is None:
            return None

        angle = euler[2]

        if angle is None:
            return None

        return float(angle)

    except Exception:
        return None