import time
import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.GP1, board.GP0)
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 test starting. Tilt/rotate the sensor and watch the numbers change.\n")

while True:
    heading, roll, pitch = sensor.euler
    accel_x, accel_y, accel_z = sensor.acceleration
    sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status

    print("Heading: {:6.1f}  Roll: {:6.1f}  Pitch: {:6.1f}  |  Accel: ({:5.2f}, {:5.2f}, {:5.2f}) m/s^2  |  Cal(sys/gyro/acc/mag): {}/{}/{}/{}".format(
        heading, roll, pitch, accel_x, accel_y, accel_z, sys_cal, gyro_cal, accel_cal, mag_cal
    ))

    time.sleep(0.2)