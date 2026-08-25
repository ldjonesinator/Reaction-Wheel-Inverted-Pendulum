import board
import analogio
import pwmio
import time

pot = analogio.AnalogIn(board.GP26)  

PWM_IN1 = pwmio.PWMOut(board.GP13, frequency=1000, duty_cycle=0)
PWM_IN2 = pwmio.PWMOut(board.GP14, frequency=1000, duty_cycle=0)

while True:
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
        
    
    print("Pot:", pot_value, "Direction:", direction, "Duty:", round(percentage, 1), "%")
    
    time.sleep(0.1)



