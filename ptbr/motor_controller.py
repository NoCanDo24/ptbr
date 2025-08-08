from gpiozero import PWMOutputDevice, OutputDevice
from ptbr.motor_encoder import Encoder
from time import sleep, time
from collections import deque

class MotorController:
    def __init__(self, in1, in2, en, p_a, p_b, KP=0.1, KD=0.05, KI=0.01):
        self.dir_1 = OutputDevice(in1)
        self.dir_2 = OutputDevice(in2)
        self.speed = PWMOutputDevice(en)
        self.en = Encoder(p_a, p_b)

        self.update_delay = 0.2
        self.target_speed = 0
        self.m_speed = 0
        self.radius = 0.0325
        self.KP = KP

        self.KD = KD
        self.prev_error = 0

        self.KI = KI
        self.sum_error = 0

    def set_speed(self, target_speed):
        if self.target_speed != float(target_speed):
            print(f"updating targetspeed! {target_speed}")
            self.target_speed = float(target_speed)
            self.speed.value = 0
            if target_speed != 0:
                self.error_buffer = deque([1,1,1,1,1,1,1,1,1,1], maxlen=10)

                


    def PID_control(self):
        if self.target_speed == 0:
            self.update_speed(0)
            return 0
        speed = self.en.get_speed() * self.radius
        error = self.target_speed - speed
        m_speed = self.m_speed + (error * self.KP) + (self.prev_error * self.KD) + (self.sum_error * self.KI)
        self.prev_error = error
        self.sum_error += error
        self.error_buffer.appendleft(abs(error))
        print(self.error_buffer)
        
        avg_error = sum(self.error_buffer)/len(self.error_buffer)

        if avg_error > 0.05:
            print(f"m_speed: {m_speed}, speed: {speed}, target speed: {self.target_speed}, error: {error}, avg_error: {avg_error}")
        
        if self.target_speed >= 0:
            m_speed = max(min(1, m_speed), 0)
        else:
            m_speed = min(max(-1, m_speed), 0)
        self.update_speed(m_speed)

        return avg_error


    def update_speed(self, speed_input):
        speed_input = float(speed_input)
        self.m_speed = speed_input
        if abs(speed_input) > 1:
            return
        self.speed.value = abs(speed_input)
        if speed_input >= 0:
            self.dir_1.value = 0
            self.dir_2.value = 1
        else:
            self.dir_1.value = 1
            self.dir_2.value = 0


if "__main__" == __name__:


    old_time = time()
    while True:
        try:
            
            r = MotorController(11, 9, 8, 5, 6, KP=0.03, KD=0.001, KI=0.00001)
            l = MotorController(10, 22, 25, 23, 24, KP=0.05, KD=0.002, KI=0.000005)


            input_speed = input("I am SPEED?").split()

            if len(input_speed) > 1:
                r.set_speed(float(input_speed[0]))
                l.set_speed(float(input_speed[1]))
            else:
                r.set_speed(float(input_speed[0]))
                l.set_speed(float(input_speed[0]))
            while True:
                try:

                    r.PID_control()
                    l.PID_control()
                    sleep(0.01)



                except KeyboardInterrupt:
                    print("New Speed")
                    break


        except KeyboardInterrupt:
            print("Program Stopped")
            break
