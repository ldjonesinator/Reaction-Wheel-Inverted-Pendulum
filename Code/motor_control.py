import board
import digitalio
import pwmio
from gpio import PIN_RES

MOTOR_FREQ = 20000

MOTOR_SW_PIN = digitalio.DigitalInOut(board.GP17)
MOTOR_SW_PIN.direction = digitalio.Direction.OUTPUT
MOTOR_SW_PIN.value = False # have it off at the start

MOTOR_PWM1 = pwmio.PWMOut(board.GP20, frequency=MOTOR_FREQ, duty_cycle=0)
MOTOR_PWM2 = pwmio.PWMOut(board.GP21, frequency=MOTOR_FREQ, duty_cycle=0)


def stop():
    MOTOR_PWM1.duty_cycle = 0
    MOTOR_PWM2.duty_cycle = 0

def turn_on(on):
    if not on:
        stop()

    MOTOR_SW_PIN.value = on

def set_speed(speed):
    if MOTOR_SW_PIN.value: # don't try if switch isn't closed
        speed = int(max(-100, min(100, speed)))
        #print(speed)
        if speed > 0:
            MOTOR_PWM1.duty_cycle = int(speed * PIN_RES / 100.0)
            MOTOR_PWM2.duty_cycle = 0

        elif speed < 0:
            MOTOR_PWM1.duty_cycle = 0
            MOTOR_PWM2.duty_cycle = int(abs(speed) * PIN_RES / 100.0)

        else:
            stop()