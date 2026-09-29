import time

import gpio
import button_control as btn

button = btn.Button()
toggle = True
i = 1
brightness = 50
control_toggle = False
gpio.switch_led_on(control_toggle)
gpio.turn_motor_on(False)
debounce = 0
motor_speed = 0

pot = analogio.AnalogIn(board.A0)

ADC_VIN = analogio.AnalogIn(board.A1)

H_BRIDGE_VL = 12 * 10 / 92
H_BRIDGE_VH = 12.5 * 10 / 92

POT_GAIN = 75
CHANGE_THRESHOLD = 200
PREVIOUS_POT = pot.value
PIN_RES = 65535


MOTOR_SW_PIN = digitalio.DigitalInOut(board.GP17)
MOTOR_SW_PIN.direction = digitalio.Direction.OUTPUT
MOTOR_SW_PIN.value = True



PWM_IN1 = pwmio.PWMOut(board.GP20, frequency=1000, duty_cycle=0)
PWM_IN2 = pwmio.PWMOut(board.GP21, frequency=1000, duty_cycle=0)

    



while True:
    button.update(gpio.check_control_switch())

    if button.check_state() == "released":
        gpio.switch_led_on(toggle)
        toggle = not toggle
    
    time.sleep(0.02)
        POT_VALUE = pot.value
    
    CHANGE = POT_VALUE - PREVIOUS_POT
    
    if CHANGE > CHANGE_THRESHOLD:
        DUTY = min(CHANGE * POT_GAIN, PIN_RES)
        
        PWM_IN1.duty_cycle = DUTY
        PWM_IN2.duty_cycle = 0
    elif CHANGE < -CHANGE_THRESHOLD :
        DUTY = min(abs(CHANGE) * POT_GAIN, PIN_RES)
        
        PWM_IN2.duty_cycle = DUTY
        PWM_IN1.duty_cycle = 0
    else:
        PWM_IN2.duty_cycle = 0
        PWM_IN1.duty_cycle = 0
    
        
    PREVIOUS_POT = POT_VALUE

    


    time.sleep(0.01)
