import time
import board
import busio
import digitalio
import pwmio
import adafruit_bno055


# Reference angle
REF_ANGLE = 180

# gains
KP = 6 #5 is good
KD = 0.01

LOOP_TIME = 0.02

# How often to print IMU/status data over serial.
PRINT_TIME = 0.2

# motor command and maximum allowed command.
MIN_PWM = 60
MAX_PWM = 100

# No safety angle.
MAX_ANGLE = 20.0

ANGLE_DEAD_BAND = 0.5

# Change value if the reaction torque is in the wrong direction.
REVERSE_MOTOR = True

PWM_FREQUENCY = 20000




# IMU pins
# GP6 = SDA
# GP7 = SCL
i2c = busio.I2C(
    scl=board.GP7,
    sda=board.GP6,
    frequency=100000
)

sensor = adafruit_bno055.BNO055_I2C(i2c)


# GP11 is active-low:
# steady on = controller active
# slow flashing = no valid IMU angle
# off = arm too far from upright
status_led = digitalio.DigitalInOut(board.GP11)
status_led.direction = digitalio.Direction.OUTPUT
status_led.value = True

# Direction feedback LEDs.
feedback_led_positive = pwmio.PWMOut(
    board.GP13,
    frequency=100,
    duty_cycle=0
)

feedback_led_negative = pwmio.PWMOut(
    board.GP12,
    frequency=100,
    duty_cycle=0
)



# High-side motor-supply switch.
motor_enable = digitalio.DigitalInOut(board.GP17)
motor_enable.direction = digitalio.Direction.OUTPUT
motor_enable.value = False

# motor driver control inputs.
motor_in1 = pwmio.PWMOut(
    board.GP20,
    frequency=PWM_FREQUENCY,
    duty_cycle=0
)

motor_in2 = pwmio.PWMOut(
    board.GP21,
    frequency=PWM_FREQUENCY,
    duty_cycle=0
)


# FUNCTIONS

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def stop_motor():
    motor_in1.duty_cycle = 0
    motor_in2.duty_cycle = 0


def disable_motor():
    stop_motor()
    motor_enable.value = False


def enable_motor():
    stop_motor()
    motor_enable.value = True
    time.sleep(0.01)


def read_euler():
    try:
        return sensor.euler
    except Exception:
        return None


def read_angle():
    #angle value

    try:
        euler = sensor.euler

        if euler is None:
            return None

        angle = euler[2]

        if angle is None:
            return None

        return float(angle)

    except Exception:
        return None


def get_angle_error(reference, measured):

    return (reference - measured + 180.0) % 360.0 - 180.0


def set_motor_speed(speed):

    speed = clamp(speed, -MAX_PWM, MAX_PWM)

    if abs(speed) < 0.5:
        stop_motor()
        return 0.0

    if abs(speed) < MIN_PWM:
        if speed > 0:
            speed = MIN_PWM
        else:
            speed = -MIN_PWM

    duty_cycle = int(
        abs(speed) * 65535 / 100.0
    )

    if speed > 0:
        motor_in2.duty_cycle = 0
        motor_in1.duty_cycle = duty_cycle

    else:
        motor_in1.duty_cycle = 0
        motor_in2.duty_cycle = duty_cycle

    return speed


def update_direction_leds(command):

    brightness = int(
        clamp(abs(command) / MAX_PWM, 0.0, 1.0)
        * 65535
    )

    if command > 0:
        feedback_led_positive.duty_cycle = brightness
        feedback_led_negative.duty_cycle = 0

    elif command < 0:
        feedback_led_positive.duty_cycle = 0
        feedback_led_negative.duty_cycle = brightness

    else:
        feedback_led_positive.duty_cycle = 0
        feedback_led_negative.duty_cycle = 0


#safe motor startup
disable_motor()
update_direction_leds(0.0)

# GP11 off during startup.
status_led.value = True

print()
print("============================================")
print("Reaction-wheel pendulum controller starting")
print("============================================")
print("REF_ANGLE:", REF_ANGLE)
print("KP:", KP, " KD:", KD)
print("Hold the arm upright and watch the 'Angle' column below --")
print("whatever it settles on is your real REF_ANGLE. It should match")
print("the current value closely; update REF_ANGLE above if it's drifted.")
print()

# Initialise Pico and IMU
time.sleep(2.0)

previous_error = 0.0
previous_time = time.monotonic()
last_print_time = previous_time

motor_running = False
command = 0.0

#Control loop
try:
    while True:
        loop_start = time.monotonic()

        dt = loop_start - previous_time
        previous_time = loop_start

        euler = read_euler()
        angle = None
        if euler is not None and euler[2] is not None:
            angle = float(euler[2])

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

        
        #Control system active
        if control_allowed:
            if not motor_running:
                enable_motor()

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

            command = set_motor_speed(
                requested_command
            )

            update_direction_leds(command)

            # GP11 steady on means control active.
            status_led.value = False

            previous_error = error
            
            
        #control system disabled
        else:
            disable_motor()
            update_direction_leds(0.0)

            command = 0.0
            motor_running = False
            previous_error = error

            if status == "ANGLE TOO LARGE":
                # GP11 off when the arm is too far from upright.
                status_led.value = True

            else:
                # Slow flashing means no valid IMU data.
                status_led.value = (
                    int(time.monotonic() * 2) % 2 == 0
                )
        
        #Print serial for debugging
        if loop_start - last_print_time >= PRINT_TIME:

            if euler is None:
                euler_text = "None"
            else:
                h = euler[0] if euler[0] is not None else float("nan")
                r = euler[1] if euler[1] is not None else float("nan")
                p = euler[2] if euler[2] is not None else float("nan")
                euler_text = "H={:.3f}, R={:.3f}, P={:.3f}".format(h, r, p)

            angle_text = "None" if angle is None else "{:.4f}".format(angle)

            print(
                "Euler:", euler_text,
                "| Angle:", angle_text,
                "| REF_ANGLE:", REF_ANGLE,
                "| Error: {:.3f}".format(error),
                "| Command: {:.1f}%".format(command),
                "| Status:", status
            )

            last_print_time = loop_start

        # Maintain approximately 50 Hz.
        elapsed = time.monotonic() - loop_start
        remaining = LOOP_TIME - elapsed

        if remaining > 0:
            time.sleep(remaining)



except Exception as fault:
    print("CONTROLLER ERROR:", fault)

    disable_motor()
    update_direction_leds(0.0)

    # Fast GP11 flashing indicates a software error.
    while True:
        status_led.value = False
        time.sleep(0.1)

        status_led.value = True
        time.sleep(0.1)