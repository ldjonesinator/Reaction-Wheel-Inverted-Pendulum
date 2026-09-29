import board
import digitalio
import analogio
import pwmio

PIN_RES = 65535
H_BRIDGE_VL = 12 * 10 / 92
H_BRIDGE_VH = 12.5 * 10 / 92

POT_PIN = analogio.AnalogIn(board.A0)

MOTOR_SW_PIN = digitalio.DigitalInOut(board.GP17)
MOTOR_SW_PIN.direction = digitalio.Direction.OUTPUT
MOTOR_SW_PIN.value = False

CONTROL_SW_PIN = digitalio.DigitalInOut(board.GP10)
CONTROL_SW_PIN.direction = digitalio.Direction.INPUT # already has pull up

SW_LED_PIN = digitalio.DigitalInOut(board.GP11)
SW_LED_PIN.direction = digitalio.Direction.OUTPUT
SW_LED_PIN.value = True

# motor pins
PWM_IN1 = pwmio.PWMOut(board.GP20, frequency=1000, duty_cycle=0)
PWM_IN2 = pwmio.PWMOut(board.GP21, frequency=1000, duty_cycle=0)

# led
PWM_LED1 = pwmio.PWMOut(board.GP13, frequency=100, duty_cycle=0)
PWM_LED2 = pwmio.PWMOut(board.GP12, frequency=100, duty_cycle=0)

ADC_VIN = analogio.AnalogIn(board.A1)


def set_motor_speed(speed):
    if MOTOR_SW_PIN.value:
        # Limit speed to -100%-100%
        speed = int(max(-100, min(100, speed)))

        # Set PWM duty cycle
        if speed > 0:
            PWM_IN1.duty_cycle = int(speed * PIN_RES / 100)
            PWM_IN2.duty_cycle = 0

        elif speed < 0:
            PWM_IN1.duty_cycle = 0
            PWM_IN2.duty_cycle = int(abs(speed) * PIN_RES / 100)

        else:
            #Stop
            PWM_IN1.duty_cycle = 0
            PWM_IN2.duty_cycle = 0

def set_control_led_brightness(led, pwm):
    pwm = int(max(-100, min(100, pwm)))
    if led == 1:
        PWM_LED1.duty_cycle = int(pwm * PIN_RES / 100)
    elif led == 2:
        PWM_LED2.duty_cycle = int(pwm * PIN_RES / 100)


def is_vin_correct():
    voltage = (ADC_VIN.value * 3.3) / PIN_RES
    return (voltage >= H_BRIDGE_VL and voltage <= H_BRIDGE_VH)

def switch_led_on(on):
    SW_LED_PIN.value = on

def turn_motor_on(on):
    MOTOR_SW_PIN.value = on

def check_control_switch():
    # pin is pull up so switch off means HIGH
    return not CONTROL_SW_PIN.value

def get_pot_percentage():
    return int(100 * POT_PIN.value / PIN_RES)