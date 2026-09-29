import board
import digitalio
import analogio
import pwmio

PIN_RES = 65535
LED_FREQ = 50
PICO_VOLT_REF = 3.3

# using voltage divider to get low and high input voltage boundaries
H_BRIDGE_VL = 12 * 10 / 92
H_BRIDGE_VH = 12.5 * 10 / 92

# adc input voltage reading from voltage divider
ADC_VIN = analogio.AnalogIn(board.A1)

CONTROL_SW_PIN = digitalio.DigitalInOut(board.GP10)
CONTROL_SW_PIN.direction = digitalio.Direction.INPUT # already has pull up

SW_LED_PIN = digitalio.DigitalInOut(board.GP11)
SW_LED_PIN.direction = digitalio.Direction.OUTPUT
SW_LED_PIN.value = False

PWM_LED1 = pwmio.PWMOut(board.GP13, frequency=LED_FREQ, duty_cycle=0)
PWM_LED2 = pwmio.PWMOut(board.GP12, frequency=LED_FREQ, duty_cycle=0)


def set_control_led_brightness(led, pwm):
    pwm = int(min(abs(pwm), 100))
    if led == 1:
        PWM_LED1.duty_cycle = int(pwm * PIN_RES / 100)
    elif led == 2:
        PWM_LED2.duty_cycle = int(pwm * PIN_RES / 100)

def is_vin_correct():
    voltage = (ADC_VIN.value * PICO_VOLT_REF) / PIN_RES
    return (voltage >= H_BRIDGE_VL and voltage <= H_BRIDGE_VH)

def switch_led_on(on):
    SW_LED_PIN.value = on

def check_control_switch():
    return not CONTROL_SW_PIN.value # pin is pull up so switch off means HIGH