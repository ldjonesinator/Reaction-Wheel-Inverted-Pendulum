import time

import gpio

i = 1
brightness = 50
control_toggle = False
gpio.switch_led_on(control_toggle)
gpio.turn_motor_on(False)
debounce = 0
motor_speed = 0


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

    


    time.sleep(0.01)