import time

import gpio
from gpio import PIN_RES
import imu_control as imu

import motor_control as motor
import button_control as btn
import pot_control as pot
from pot_control import POT_GAIN

# Third Euler value when the arm is exactly upright.
REF_ANGLE = 0
EULER_ANGLE_DIMENSION = 1


LOOP_TIME = 0.02
ANGLE_LIMIT = 90.0


KP = 1.0 #NOT 2
KI = 1.0 # must be > 1
KD = 0.0


# The integral term can never contribute more than this many percent
# (anti-windup).
INTEGRAL_LIMIT = 100.0

# Derivative low-pass filter: 1.0 = raw derivative, smaller = smoother.
# The BNO055 angle is quantised to 1/16 degree, so raw derivatives are
# noisy.
DERIV_FILTER = 0.15

# No motor output while the error is inside this many degrees.
ANGLE_DEAD_BAND = 0.10

# ----- Target-angle dithering (Brick's ANGLE_FIXRATE) -------------
# Each loop the target angle is nudged away from the measured angle,
# which keeps the error (and so the wheel acceleration) from settling at
# zero. Brick uses 1.0 deg/s. Leave at 0.0 (off) until P and D work.
ANGLE_FIXRATE = 0.0
# The target can never move further than this from upright (degrees).
TARGET_LIMIT = 10.0

MIN_PWM = 0.0
MAX_PWM = 100.0

# True:  a 0-100% command is stretched over MIN_PWM..MAX_PWM, so small
#        commands give small (but moving) output.
# False: old behaviour - anything below MIN_PWM is lifted to MIN_PWM.
DEADZONE_REMAP = True

REVERSE_MOTOR = False
PWM_FREQUENCY = 20000


USER_MODES = {"MANUAL": 0, "PID": 1}

mode_btn = btn.Button()
user_pot = pot.Pot() # XD


def input_volt_check(was_motor_on):
    # turn on HSS if wasn't on already, vice versa
    if gpio.is_vin_correct() and not was_motor_on:
        motor.turn_on(True)
        return True
    elif not gpio.is_vin_correct() and was_motor_on:
        motor.turn_on(False)
        return False

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def wrap_angle(angle):
    """Wrap an angle difference into -180...+180 degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def set_clamped_motor_speed(command):

    command = clamp(command, -100.0, 100.0)
    magnitude = abs(command)

    if magnitude < 0.5:
        motor.stop()
        return 0.0

    if DEADZONE_REMAP:
        duty_percent = MIN_PWM + (MAX_PWM - MIN_PWM) * magnitude / 100.0
    else:
        duty_percent = clamp(magnitude, MIN_PWM, MAX_PWM)

    duty_percent = clamp(duty_percent, 0.0, MAX_PWM)

    motor.set_speed(duty_percent)
    if command > 0:
        return duty_percent

    return -duty_percent



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

motor_running = input_volt_check(motor_running)
update_direction_leds(0.0)

time.sleep(1.0) # delay for initialisation of io and user

command = 0.0
tilt = None                # degrees from upright (+ when angle > REF_ANGLE)
target_angle = 0.0         # degrees from upright, moved by dithering
error = 0.0
previous_error = 0.0
integral = 0.0
derivative = 0.0
p_term = 0.0
i_term = 0.0
d_term = 0.0

previous_ns = time.monotonic_ns()

while True:
    motor_running = input_volt_check(motor_running)

    mode_btn.update(gpio.check_control_switch())

    if mode_btn.check_state() == "released":
        # user_mode = (user_mode + 1) % len(USER_MODES) # rotate through modes
        # gpio.switch_led_on(user_mode % 2 == 1)
        if user_mode != USER_MODES["PID"]:
            user_mode = USER_MODES["PID"]
            gpio.switch_led_on(True)
        else:
            USER_MODES["MANUAL"]
            # gpio.switch_led_on(False)

        # resetting everyting
        user_pot.reset()
        command = 0.0
        previous_error = 0.0
        previous_ns = time.monotonic()
        motor.stop()
        update_direction_leds(0.0)
        target_angle = 0.0
        integral = 0.0
        derivative = 0.0
        error = 0.0
        p_term = 0.0
        i_term = 0.0
        d_term = 0.0


    if user_mode == USER_MODES["MANUAL"]:
        pot_change = user_pot.get_pot_change()
        speed = int(100 * pot_change * POT_GAIN / PIN_RES)
        if speed != prev_speed:
            motor.set_speed(speed)
            prev_speed = speed

        time.sleep(0.01)


    elif user_mode == USER_MODES["PID"]:
        now_ns = time.monotonic_ns()
        dt = (now_ns - previous_ns) / 1e9
        previous_ns = now_ns

        angle = imu.read_angle()

        if angle is None:
            tilt = None
            control_allowed = False
            status = "NO IMU DATA"

        else:
            tilt = wrap_angle(angle - REF_ANGLE)
            control_allowed = abs(tilt) <= ANGLE_LIMIT

            if control_allowed:
                status = "CONTROL ACTIVE"
            else:
                status = "ANGLE TOO LARGE"


        if control_allowed:

            # Dither the target away from the measured angle.
            if ANGLE_FIXRATE > 0.0:
                if tilt < target_angle:
                    target_angle += ANGLE_FIXRATE * dt
                else:
                    target_angle -= ANGLE_FIXRATE * dt

                target_angle = clamp(
                    target_angle, -TARGET_LIMIT, TARGET_LIMIT
                )

            error = tilt - target_angle

            if 0.0 < dt < 0.1:
                # Filtered derivative of the error.
                raw_derivative = (error - previous_error) / dt
                derivative += DERIV_FILTER * (raw_derivative - derivative)

                # Integral with anti-windup clamp.
                if KI > 0.0 and abs(error) > ANGLE_DEAD_BAND:
                    integral += error * dt
                    limit = INTEGRAL_LIMIT / KI
                    integral = clamp(integral, -limit, limit)

            p_term = KP * error
            i_term = KI * integral
            d_term = KD * derivative

            if abs(error) <= ANGLE_DEAD_BAND:
                requested_command = 0.0
            else:
                requested_command = p_term + i_term + d_term

            if REVERSE_MOTOR:
                requested_command = -requested_command

            command = set_clamped_motor_speed(requested_command)

            update_direction_leds(command)

            previous_error = error


        else:
            motor.stop()
            update_direction_leds(0.0)

            command = 0.0
            target_angle = 0.0
            integral = 0.0
            derivative = 0.0
            error = 0.0
            p_term = 0.0
            i_term = 0.0
            d_term = 0.0

        # Maintain the loop period.
        elapsed = (time.monotonic_ns() - now_ns) / 1e9
        remaining = LOOP_TIME - elapsed

        if remaining > 0:
            time.sleep(remaining)
