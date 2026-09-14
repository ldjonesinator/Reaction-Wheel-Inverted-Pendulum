import board
import digitalio
import time
import pwmio
import analogio

PIN_RES = 65535

MOTOR_SW_PIN = digitalio.DigitalInOut(board.GP17)
MOTOR_SW_PIN.direction = digitalio.Direction.OUTPUT
MOTOR_SW_PIN.value = False

LED_PIN = digitalio.DigitalInOut(board.GP11)
LED_PIN.direction = digitalio.Direction.OUTPUT
LED_PIN.value = True

PWM_IN1 = pwmio.PWMOut(board.GP20, frequency=1000, duty_cycle=0)
PWM_IN2 = pwmio.PWMOut(board.GP21, frequency=1000, duty_cycle=0)

ADC_MOTOR = analogio.AnalogIn(board.A1)

H_BRIDGE_VL = 12 * 10 / 92
H_BRIDGE_VH = 12.5 * 10 / 92


def set_speed(speed):

    # Limit speed to -100%-100%
    speed = max(-100, min(100, speed))

    # Set PWM duty cycle
    if speed > 0:
        #Forward
        PWM_IN2.duty_cycle = 0
        PWM_IN1.duty_cycle = int(speed * PIN_RES / 100)

    elif speed < 0:
        #Reverse
        PWM_IN1.duty_cycle = 0
        PWM_IN2.duty_cycle = int(abs(speed) * PIN_RES / 100)

    else:
        #Stop
        PWM_IN1.duty_cycle = 0
        PWM_IN2.duty_cycle = 0


def get_voltage():
    voltage = (ADC_MOTOR.value * 3.3) / PIN_RES
    return (voltage >= H_BRIDGE_VL and voltage <= H_BRIDGE_VH)

def turn_led_on(on):
    LED_PIN.value = on




while True:

    if get_voltage():
        turn_led_on(False)
        MOTOR_SW_PIN.value = True
    else:
        turn_led_on(True)
        MOTOR_SW_PIN.value = False



    set_speed(25)
    time.sleep(10)

    set_speed(0)
    time.sleep(2)

    set_speed(-25)
    time.sleep(10)

    set_speed(0)
    time.sleep(2)
