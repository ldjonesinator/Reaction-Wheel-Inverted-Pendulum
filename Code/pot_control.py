import board
import analogio
from gpio import PIN_RES


POT_GAIN = 75
CHANGE_THRESHOLD = 200


POT_PIN = analogio.AnalogIn(board.A0)


class Pot:
    def __init__(self):
        self.reset()

    def reset(self):
        self.prev_val = POT_PIN.value

    def get_pot_percentage(self):
        return int(100 * POT_PIN.value / PIN_RES)

    def get_pot_change(self):
        change = POT_PIN.value - self.prev_val
        self.prev_val = POT_PIN.value
        return change