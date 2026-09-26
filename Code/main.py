import time

import gpio
import button_control as btn

button = btn.Button()
toggle = True

while True:
    button.update(gpio.check_control_switch())

    if button.check_state() == "released":
        gpio.switch_led_on(toggle)
        toggle = not toggle
    
    time.sleep(0.02)