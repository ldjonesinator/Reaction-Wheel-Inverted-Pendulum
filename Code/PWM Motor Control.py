import board
import digitalio
import time
import pwmio

PWM_IN1 = pwmio.PWMOut(board.GP13, frequency=1000, duty_cycle=0)
PWM_IN2 = pwmio.PWMOut(board.GP14, frequency=1000, duty_cycle=0)



def set_motor(speed):


    # Limit speed to -100%-100%
    speed = max(-100, min(100, speed))

    # Set PWM duty cycle
    if speed > 0:
        #Forward
        PWM_IN2.duty_cycle = 0
        PWM_IN1.duty_cycle = int(speed * 65535 / 100)
        
    elif speed < 0:
        #Reverse
        PWM_IN1.duty_cycle = 0
        PWM_IN2.duty_cycle = int(abs(speed) * 65535 / 100)
    
    else:
        #Stop
        PWM_IN1.duty_cycle = 0
        PWM_IN2.duty_cycle = 0
        
        



while True:

    print("Forward 70%")
    set_motor(70)
    time.sleep(3)
    
    print("Reverse 70%")
    set_motor(90)
    time.sleep(3)
    
    print("Stop")
    set_motor(0)
    time.sleep(2)
