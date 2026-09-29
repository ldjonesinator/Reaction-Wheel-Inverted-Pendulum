import time

import gpio
from gpio import PIN_RES
import motor_control as motor
import pot_control as pot
from pot_control import POT_GAIN
import button_control as btn


button = btn.Button()
user_pot = pot.Pot() # XD
prev_speed = 0
was_motor_on = False


def input_volt_check():
    # turn on HSS if wasn't on already, vice versa
    if gpio.is_vin_correct() and not was_motor_on:
        motor.turn_on(True)
        was_motor_on = True
    elif not gpio.is_vin_correct() and was_motor_on:
        motor.turn_on(False)
        was_motor_on = False


while True:
    input_volt_check()

    pot_change = user_pot.get_pot_change()
    speed = int(100 * pot_change * POT_GAIN / PIN_RES)
    if speed != prev_speed:
        motor.set_speed(speed)
        prev_speed = speed

    time.sleep(0.01)
