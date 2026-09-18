import time
import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.GP1, board.GP0)
sensor = adafruit_bno055.BNO055_I2C(i2c)

def get_angle_data():
	# heading, roll, pitch
	return sensor.euler

def get_accel_data():
	# accel_x, accel_y, accel_z
	return sensor.acceleration

def get_cal_data():
	# sys_cal, gyro_cal, accel_cal, mag_cal
	return sensor.calibration_status

if __name__ == "__main__":
	while True:
		heading, roll, pitch = get_angle_data()
		accel_x, accel_y, accel_z = get_accel_data()
		sys_cal, gyro_cal, accel_cal, mag_cal = get_cal_data()

		print("Heading: {:6.1f}  Roll: {:6.1f}  Pitch: {:6.1f}  |  Accel: ({:5.2f}, {:5.2f}, {:5.2f}) m/s^2  |  Cal(sys/gyro/acc/mag): {}/{}/{}/{}".format(
			heading, roll, pitch, accel_x, accel_y, accel_z, sys_cal, gyro_cal, accel_cal, mag_cal
		))

		time.sleep(0.2)