# code.py
# Reaction-wheel inverted pendulum controller
# Raspberry Pi Pico + BNO055 + DRV8231 (CircuitPython)
#
# Adapted from the Brick Experiment Channel reaction-wheel pendulum
# (Raspberry Pi + MPU9250 + TB6612FNG + EV3 motor).
#
# Carried over from that project:
#   - PID on the arm angle error (with integral term)
#   - dithering of the target angle (their ANGLE_FIXRATE)
#   - motor stops if the arm tips past a limit angle
#   - data logged during the run, dumped afterwards for analysis
#
# Not carried over (this PCB has no wheel encoder / no rise-up torque):
#   - wheel angular velocity, back-EMF compensation, wheel-speed bleed-off
#   - the rise-up sequence (the arm is placed upright by hand)
#
# The BNO055 fuses accelerometer + gyro on-chip, so it replaces their
# software complementary filter.

import time
import board
import busio
import digitalio
import pwmio
import adafruit_bno055
from array import array


# ============================================================
# SETTINGS
# ============================================================

# Third Euler value when the arm is exactly upright.
REF_ANGLE = 0
EULER_ANGLE_DIMENSION = 1


# Control loop period. The BNO055 only produces a new fused angle every
# 10 ms, so running faster than 100 Hz just re-reads the same sample.
LOOP_TIME = 0.02

# Motor is disabled (and the run ends) beyond this angle from upright.
ANGLE_LIMIT = 90.0

# ----- PID gains --------------------------------------------------
# Angles are in degrees from upright. The PID output is a motor command
# in percent (-100...100).
#   KP: percent per degree of error
#   KI: percent per (degree * second) of accumulated error
#   KD: percent per (degree / second) of error rate
# Tune in the order P -> D -> I.
KP = 1.0 #NOT 2
KI = 1.0#1 
KD = 0.0   # was 0.0 -- P alone is undamped and will oscillate until it
           # falls, this is a starting point to tune from, not a final value

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

# ----- Motor ------------------------------------------------------
# Lowest duty (percent) that reliably turns the loaded motor, and the
# highest duty allowed.
MIN_PWM = 0.0
MAX_PWM = 100.0

# True:  a 0-100% command is stretched over MIN_PWM..MAX_PWM, so small
#        commands give small (but moving) output.
# False: old behaviour - anything below MIN_PWM is lifted to MIN_PWM.
DEADZONE_REMAP = True   # was False -- small errors were slamming
                        # straight to 70% instead of scaling smoothly

# Change if the reaction torque pushes the arm the wrong way.
REVERSE_MOTOR = False

PWM_FREQUENCY = 20000

# ----- Serial output ----------------------------------------------
PRINT_LIVE = True         # continuous status line
PRINT_TIME = 0.2          # seconds between status lines

# Record each run and print it as CSV when the run ends (arm falls or
# leaves ANGLE_LIMIT), so nothing has to be read while balancing.
LOG_ENABLED = False
LOG_SECONDS = 20.0        # most recent seconds kept (ring buffer)
LOG_MIN_RUN = 2.0         # runs shorter than this are not dumped

# ----- IMU --------------------------------------------------------
# True: accelerometer + gyro only (no magnetometer), so the motor's
# magnetic field cannot disturb the fused angle. REF_ANGLE must be
# measured again after changing this.
USE_IMUPLUS_MODE = False


# ============================================================
# BNO055
# ============================================================

# GP6 = SDA, GP7 = SCL
i2c = busio.I2C(
    scl=board.GP7,
    sda=board.GP6,
    frequency=100000
)

sensor = adafruit_bno055.BNO055_I2C(i2c)

if USE_IMUPLUS_MODE:
    sensor.mode = adafruit_bno055.IMUPLUS_MODE


# ============================================================
# STATUS AND DIRECTION LEDS
# ============================================================

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


# ============================================================
# MOTOR HARDWARE
# ============================================================

# High-side motor-supply switch.
motor_enable = digitalio.DigitalInOut(board.GP17)
motor_enable.direction = digitalio.Direction.OUTPUT
motor_enable.value = False

# DRV8231 control inputs.
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


# ============================================================
# RUN LOG (ring buffer, dumped as CSV when a run ends)
# ============================================================

LOG_NAMES = (
    "time_s", "tilt_deg", "target_deg", "error_deg",
    "P_pct", "I_pct", "D_pct", "cmd_pct",
)
LOG_SIZE = int(LOG_SECONDS / LOOP_TIME + 0.5)
log_buffers = []
log_index = 0
log_count = 0

if LOG_ENABLED:
    for _ in LOG_NAMES:
        log_buffers.append(array("f", [0.0] * LOG_SIZE))


def log_reset():
    global log_index, log_count
    log_index = 0
    log_count = 0


def log_sample(values):
    global log_index, log_count

    if not LOG_ENABLED:
        return

    for channel in range(len(values)):
        log_buffers[channel][log_index] = values[channel]

    log_index = (log_index + 1) % LOG_SIZE

    if log_count < LOG_SIZE:
        log_count += 1


def dump_log():
    if not LOG_ENABLED or log_count == 0:
        return

    start = (log_index - log_count) % LOG_SIZE

    print()
    print("===== RUN LOG: {} rows =====".format(log_count))
    print(",".join(LOG_NAMES))

    for k in range(log_count):
        i = (start + k) % LOG_SIZE
        print(
            "{:.3f},{:.2f},{:.2f},{:.2f},{:.1f},{:.1f},{:.1f},{:.1f}".format(
                *[buffer[i] for buffer in log_buffers]
            )
        )

    print("===== END OF LOG =====")
    print()


# ============================================================
# FUNCTIONS
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def wrap_angle(angle):
    """Wrap an angle difference into -180...+180 degrees."""
    return (angle + 180.0) % 360.0 - 180.0


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
    """Return the (heading, roll, pitch) tuple, or None on failure."""
    try:
        return sensor.euler
    except Exception:
        return None


def drive_motor(command):
    """
    Drive the motor from a signed command in percent (-100...100).

    Positive: PWM on GP20.   Negative: PWM on GP21.
    Returns the signed duty (percent) actually applied.
    """

    command = clamp(command, -100.0, 100.0)
    magnitude = abs(command)

    if magnitude < 0.5:
        stop_motor()
        return 0.0

    if DEADZONE_REMAP:
        duty_percent = MIN_PWM + (MAX_PWM - MIN_PWM) * magnitude / 100.0
    else:
        duty_percent = clamp(magnitude, MIN_PWM, MAX_PWM)

    duty_percent = clamp(duty_percent, 0.0, MAX_PWM)
    duty_cycle = int(duty_percent * 65535 / 100.0)

    if command > 0:
        motor_in2.duty_cycle = 0
        motor_in1.duty_cycle = duty_cycle
        return duty_percent

    motor_in1.duty_cycle = 0
    motor_in2.duty_cycle = duty_cycle
    return -duty_percent


def update_direction_leds(command):
    """
    GP13 indicates a positive motor command.
    GP12 indicates a negative motor command.
    """

    brightness = int(
        clamp(abs(command) / 100.0, 0.0, 1.0)
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


# ============================================================
# SAFE STARTUP
# ============================================================

disable_motor()
update_direction_leds(0.0)

# GP11 off during startup.
status_led.value = True

print()
print("============================================")
print("Reaction-wheel pendulum controller starting")
print("============================================")
print("REF_ANGLE:", REF_ANGLE)
print("KP:", KP, " KI:", KI, " KD:", KD)
print("Dithering (ANGLE_FIXRATE):", ANGLE_FIXRATE, "deg/s")
print("Minimum PWM:", MIN_PWM, " Maximum PWM:", MAX_PWM)
print("Maximum control angle:", ANGLE_LIMIT)
print("Hold the arm upright and watch the 'Angle' value below --")
print("it should match REF_ANGLE. Update REF_ANGLE if it has drifted.")
print()

# Allow Pico and BNO055 to initialise.
time.sleep(2.0)

PRINT_NS = int(PRINT_TIME * 1e9)

previous_ns = time.monotonic_ns()
last_print_ns = previous_ns
run_start_ns = previous_ns

motor_running = False
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


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

try:
    while True:
        now_ns = time.monotonic_ns()
        dt = (now_ns - previous_ns) / 1e9
        previous_ns = now_ns

        euler = read_euler()
        angle = None
        if euler is not None and euler[EULER_ANGLE_DIMENSION] is not None:
            angle = float(euler[EULER_ANGLE_DIMENSION])

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

        # ----------------------------------------------------
        # CONTROL ACTIVE
        # ----------------------------------------------------

        if control_allowed:

            if not motor_running:
                enable_motor()

                # Fresh run: clear all controller state.
                target_angle = 0.0
                integral = 0.0
                derivative = 0.0
                previous_error = tilt - target_angle
                run_start_ns = now_ns
                log_reset()

                motor_running = True
                print("Motor enabled")

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

            command = drive_motor(requested_command)

            update_direction_leds(command)

            # GP11 steady on means control active.
            status_led.value = False

            log_sample((
                (now_ns - run_start_ns) / 1e9,
                tilt, target_angle, error,
                p_term, i_term, d_term, command,
            ))

            previous_error = error

        # ----------------------------------------------------
        # CONTROL DISABLED
        # ----------------------------------------------------

        else:
            disable_motor()
            update_direction_leds(0.0)

            if motor_running:
                run_seconds = (now_ns - run_start_ns) / 1e9
                print(
                    "Motor disabled:", status,
                    "| run lasted {:.1f} s".format(run_seconds)
                )

                if LOG_ENABLED and run_seconds >= LOG_MIN_RUN:
                    dump_log()

                motor_running = False

            command = 0.0
            target_angle = 0.0
            integral = 0.0
            derivative = 0.0
            error = 0.0
            p_term = 0.0
            i_term = 0.0
            d_term = 0.0

            if status == "ANGLE TOO LARGE":
                # GP11 off when the arm is too far from upright.
                status_led.value = True

            else:
                # Slow flashing means no valid IMU data.
                status_led.value = (
                    int(time.monotonic() * 2) % 2 == 0
                )

        # ----------------------------------------------------
        # LIVE SERIAL STATUS
        # ----------------------------------------------------

        if PRINT_LIVE and (now_ns - last_print_ns) >= PRINT_NS:
            last_print_ns = now_ns

            if euler is None:
                euler_text = "None"
            else:
                euler_text = "H={}, R={}, P={}".format(
                    "{:.2f}".format(euler[0]) if euler[0] is not None else "-",
                    "{:.2f}".format(euler[1]) if euler[1] is not None else "-",
                    "{:.2f}".format(euler[2]) if euler[2] is not None else "-",
                )

            angle_text = "None" if angle is None else "{:.3f}".format(angle)
            tilt_text = "None" if tilt is None else "{:+.2f}".format(tilt)

            try:
                cal = sensor.calibration_status
                cal_text = "{}/{}/{}/{}".format(cal[0], cal[1], cal[2], cal[3])
            except Exception:
                cal_text = "?"

            print(
                "Euler:", euler_text,
                "| Angle:", angle_text,
                "| Tilt:", tilt_text,
                "| Target: {:+.2f}".format(target_angle),
                "| Error: {:+.2f}".format(error),
                "| P/I/D: {:+.0f}/{:+.0f}/{:+.0f}".format(
                    p_term, i_term, d_term
                ),
                "| Cmd: {:+.0f}%".format(command),
                "| Cal(sys/gyro/acc/mag):", cal_text,
                "| Status:", status
            )

        # Maintain the loop period.
        elapsed = (time.monotonic_ns() - now_ns) / 1e9
        remaining = LOOP_TIME - elapsed

        if remaining > 0:
            time.sleep(remaining)


# ============================================================
# FAULT SHUTDOWN
# ============================================================

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