import time

import gpio
from gpio import PIN_RES
import imu_control as imu

import motor_control as motor
import button_control as btn
import pot_control as pot
from pot_control import POT_GAIN



KP = 6 #5 is good
KD = 0.01
REF_ANGLE = 180

# motor command
MIN_PWM = 60
MAX_PWM = 100
MAX_ANGLE = 20.0 # Doesn't work if goes over this from ref
ANGLE_DEAD_BAND = 0.5
REVERSE_MOTOR = True

LOOP_TIME = 0.02
PRINT_TIME = 0.2
PWM_FREQUENCY = 20000


USER_MODES = {"MANUAL": 0, "PID": 1}

mode_btn = btn.Button()
user_pot = pot.Pot() # XD


def input_volt_check(was_motor_on):
    # turn on HSS if wasn't on already, vice versa
    if gpio.is_vin_correct() and not was_motor_on:
        motor.turn_on(True)
        was_motor_on = True
    elif not gpio.is_vin_correct() and was_motor_on:
        motor.turn_on(False)
        was_motor_on = False

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def get_angle_error(reference, measured):
    return (reference - measured + 180.0) % 360.0 - 180.0


def set_clamped_motor_speed(speed):
    speed = clamp(speed, -MAX_PWM, MAX_PWM)

    if abs(speed) < 0.5:
        motor.stop()
        return 0.0

    if abs(speed) < MIN_PWM:
        if speed > 0:
            speed = MIN_PWM
        else:
            speed = -MIN_PWM

    motor.set_speed(speed)

    return speed


def update_direction_leds(command):
    brightness = int(
        clamp(abs(command) / MAX_PWM, 0.0, 1.0)
        * 65535
    )

    if command > 0:
        gpio.set_control_led_brightness(1, brightness)
        gpio.set_control_led_brightness(2, 0)

    elif command < 0:
        gpio.set_control_led_brightness(1, 0)
        gpio.set_control_led_brightness(2, brightness)

    else:
        gpio.set_control_led_brightness(1, 0)
        gpio.set_control_led_brightness(2, 0)


motor_running = False
prev_speed = 0
user_mode = USER_MODES["MANUAL"]

input_volt_check(motor_running)
update_direction_leds(0.0)

time.sleep(3.0) # delay for initialisation of io and user

command = 0.0
previous_error = 0.0
previous_time = time.monotonic()
last_print_time = previous_time

while True:
    input_volt_check()

    mode_btn.update(gpio.check_control_switch())

    if mode_btn.check_state() == "released":
        user_mode = (user_mode + 1) % len(USER_MODES) # rotate through modes
        gpio.switch_led_on(user_mode % 2 == 1)

        # resetting everyting
        user_pot.reset()
        command = 0.0
        previous_error = 0.0
        previous_time = time.monotonic()
        last_print_time = previous_time
        update_direction_leds(0.0)
        motor.stop()



    if user_mode == USER_MODES["MANUAL"]:
        pot_change = user_pot.get_pot_change()
        speed = int(100 * pot_change * POT_GAIN / PIN_RES)
        if speed != prev_speed:
            motor.set_speed(speed)
            prev_speed = speed

        time.sleep(0.01)


    elif user_mode == USER_MODES["PID"]:
        loop_start = time.monotonic()

        dt = loop_start - previous_time
        previous_time = loop_start

        angle = imu.read_angle()

        if angle is None:
            error = 0.0
            control_allowed = False
            status = "NO IMU DATA"

        else:
            error = get_angle_error(
                REF_ANGLE,
                angle
            )

            control_allowed = (
                abs(error) <= MAX_ANGLE
            )

            if control_allowed:
                status = "CONTROL ACTIVE"
            else:
                status = "ANGLE TOO LARGE"


        # Control system active
        if control_allowed:
            if not motor_running:
                motor.turn_on(True)

                # Prevent a derivative spike during startup.
                previous_error = error
                motor_running = True

            if 0.0 < dt < 0.1:
                derivative = (
                    error - previous_error
                ) / dt
            else:
                derivative = 0.0

            if abs(error) <= ANGLE_DEAD_BAND:
                requested_command = 0.0
            else:
                requested_command = (
                    KP * error
                    + KD * derivative
                )

            if REVERSE_MOTOR:
                requested_command = -requested_command

            command = set_clamped_motor_speed(
                requested_command
            )

            update_direction_leds(command)


            previous_error = error


        #control system disabled
        else:
            motor.turn_on(False)
            update_direction_leds(0.0)

            command = 0.0
            motor_running = False
            previous_error = error

        # Maintain approximately 50 Hz.
        elapsed = time.monotonic() - loop_start
        remaining = LOOP_TIME - elapsed

        if remaining > 0:
            time.sleep(remaining)
