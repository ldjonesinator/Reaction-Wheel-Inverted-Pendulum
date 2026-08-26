import time
import imu_control as imu
import PWM_motor_control as motor


REF_ANGLE = 90 # calibrated pendulum angle
KP = 1
KD = 0.1
KI = 0.2
DELAY = 0.1

def PID_controller(angle, ref, prev_error, integral, dt, kp, kd, ki):
	if round(dt) <= 0:
		return 0, prev_error, integral

	error = ref - angle
	derivative = (error - prev_error) / dt # "de/dt"
	integral += error * dt

	control = kp * error + kd * derivative + ki * integral
	return control, error, integral

# test loop
if __name__ == "__main__":
	error = 0
	integral = 0
	prev_time = time.time_ns() // 1_000_000
	while True:
		heading, roll, pitch = imu.get_angle_data()
		dt = time.time_ns() // 1_000_000 - prev_time
		control, error, integral = PID_controller(heading, REF_ANGLE, error, integral,
												  dt, KP, KD, KI)

		motor.set_speed(control) # have to change this after testing

		prev_time = time.time_ns() // 1_000_000
		time.sleep(DELAY)
