import RPi.GPIO as GPIO
import time

# import the library
from RpiMotorLib import RpiMotorLib
    
#define GPIO pins
GPIO_pins = (14, 15, 18) # Microstep Resolution MS1-MS3 -> GPIO Pin
direction= 20       # Direction -> GPIO Pin
step = 21      # Step -> GPIO Pin
i=0 
# Declare an named instance of class pass GPIO pins numbers
mymotortest = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "A4988")


# call the function, pass the arguments
for i in range(0,1):
    
    mymotortest.motor_go(True, "Full" , 260, .05, False, .05) 
    time.sleep(10)
    i=i+1
    
  


