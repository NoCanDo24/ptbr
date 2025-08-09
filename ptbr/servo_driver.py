from gpiozero import PWMOutputDevice

import lgpio
from time import sleep
import numpy as np


class Servo:

    def __init__(self, pin, frequency=50, min_cycle=0.5, max_cycle=2.5, angle_range=270):
        self.pin = pin
        self.pwm = PWMOutputDevice(pin)

        # self.h = lgpio.gpiochip_open(4)
        # lgpio.pwm_frequency(self.h, self.pin, frequency)

        self.frequency = frequency
        self.pwm.frequency = frequency

        self.min = min_cycle
        self.max = max_cycle

        self.angle_range = angle_range
        
    
    def updateAngle(self, angle, steps=0.001, delay=0.1):
        ms = angle * (self.max - self.min)/self.angle_range + self.min

        target_pwm = self.msToValue(ms)


        self.pwm.value = target_pwm

        # pwm_g = current_pwm
        # if(current_pwm < target_pwm):

        #     while pwm_g < target_pwm:
            
        #         if pwm_g + steps > target_pwm:
        #             pwm_g = target_pwm
        #         else:
        #             pwm_g += steps
        #         self.pwm.value = pwm_g
        #         sleep(delay)
        #         print(current_pwm, pwm_g, target_pwm)
        # else:
        #     while pwm_g > target_pwm:
            
        #         if pwm_g - steps < target_pwm:
        #             pwm_g = target_pwm
        #         else:
        #             pwm_g -= steps
        #         self.pwm.value = pwm_g
        #         sleep(delay)
        #         print(current_pwm, pwm_g, target_pwm)

        
    def msToValue(self, ms):
        res = ms*0.001*self.frequency
        return res

if "__main__" == __name__:

    pin = int(input("On What Pin do you want it?"))
    s = ServoMotor(pin)
    while True:
        try:
            angleInput = int(input("What angle do you want?"))
            s.updateAngle(angleInput)



        except KeyboardInterrupt:
            print("Program Stopped")
            break
