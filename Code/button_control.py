NUMBER_OF_POLLS = 4

class Button:

    def __init__(self, state=False, new_state_count=0):
        self.state = state
        self.new_state_count = new_state_count
        self.normal_state = self.state
        self.changed = False

    def update(self, new_state):
        if new_state != self.state:
            self.new_state_count += 1
            if self.new_state_count >= NUMBER_OF_POLLS:
                self.state = new_state
                self.changed = True
                self.new_state_count = 0
        else:
            self.new_state_count = 0

    def check_state(self):
        if self.changed:
            self.changed = False
            if self.state == self.normal_state:
                return "released"
            else:
                return "pushed"



