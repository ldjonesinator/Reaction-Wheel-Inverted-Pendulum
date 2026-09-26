import time

import gpio

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
CHANGE_THRESHOLD = 1
PREVIOUS_POT = pot.value
PIN_RES = 65535


MOTOR_SW_PIN = digitalio.DigitalInOut(board.GP17)
MOTOR_SW_PIN.direction = digitalio.Direction.OUTPUT
MOTOR_SW_PIN.value = True



PWM_IN1 = pwmio.PWMOut(board.GP20, frequency=1000, duty_cycle=0)
PWM_IN2 = pwmio.PWMOut(board.GP21, frequency=1000, duty_cycle=0)

    



while True:
    if gpio.is_vin_correct():
        gpio.turn_motor_on(True)
    else:
        gpio.turn_motor_on(False)

    if debounce <= 0:
        if gpio.check_control_switch():
            control_toggle = not control_toggle
            gpio.switch_led_on(control_toggle)
            debounce = 50
    else:
        debounce -= 1


    if control_toggle:
        if motor_speed != gpio.get_pot_percentage() * 2 - 100:
            motor_speed = gpio.get_pot_percentage() * 2 - 100
            gpio.set_motor_speed(motor_speed)
    else:
        gpio.set_motor_speed(0) # motor control with pot
        gpio.set_control_led_brightness(1, (brightness + i) % 100)
        gpio.set_control_led_brightness(2, (brightness - i) % 100)
        i += 1

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
