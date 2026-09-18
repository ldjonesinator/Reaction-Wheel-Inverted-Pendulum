import board
import digitalio
import time
import pwmio
import analogio

import gpio

i = 1
brightness = 50

while True:
    gpio.set_control_led_brightness(1, (brightness + i) % 100)
    gpio.set_control_led_brightness(2, (brightness - i) % 100)
    i += 1

    if gpio.check_control_switch():
        gpio.switch_led_on(True)
    else:
        gpio.switch_led_on(False)


    time.sleep(0.1)