import board
import digitalio
import time
import pwmio
import analogio

pot = analogio.AnalogIn(board.A0)

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

    pot_value = pot.value
    
    if pot_value > 32768:
        PWM_IN1.duty_cycle = min((pot_value - 32768) * 2, 65535)
        PWM_IN2.duty_cycle = 0
        direction = "forward"
        percentage = (PWM_IN1.duty_cycle / 65535) * 100
    else:
        PWM_IN2.duty_cycle = min((32768 - pot_value) * 2, 65535)
        PWM_IN1.duty_cycle = 0
        direction = "reverse"
        percentage = (PWM_IN2.duty_cycle / 65535) * 100
        
    
    time.sleep(0.1)



