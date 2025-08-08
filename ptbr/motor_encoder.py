from gpiozero import DigitalInputDevice
from signal import pause
from time import time, sleep
import math
class Encoder:

    def __init__(self, pin_a, pin_b):
        self.phase_a = DigitalInputDevice(pin_a)
        self.phase_b = DigitalInputDevice(pin_b)

        self.phase_a.when_activated = self.trig_a
        self.phase_a.when_deactivated = self.trig_a
        self.phase_b.when_activated = self.trig_b
        self.phase_b.when_deactivated = self.trig_b

        self.pulse_count = 0
        self.ppr_total = 2184

        self.last_pulse_count = 0
        self.last_time = time()

    def get_speed(self):
        current_time = time()

        delta_pulses = self.pulse_count - self.last_pulse_count
        delta_time = current_time - self.last_time

        if delta_time == 0 or delta_pulses == 0:
            return 0.0

        angular_speed = (delta_pulses/self.ppr_total) * 2 * math.pi / delta_time

        self.last_pulse_count = self.pulse_count
        self.last_time = current_time

        return angular_speed


    def trig_a(self):
        if self.phase_a.is_active:
            if self.phase_b.is_active:
                self.pulse_count -= 1
            else:
                self.pulse_count += 1
        else:
            if self.phase_b.is_active:
                self.pulse_count += 1
            else:
                self.pulse_count -= 1

    def trig_b(self):
        if self.phase_b.is_active:
            if self.phase_a.is_active:
                self.pulse_count += 1
            else:
                self.pulse_count -= 1
        else:
            if self.phase_a.is_active:
                self.pulse_count -= 1
            else:
                self.pulse_count += 1




if __name__ == "__main__":
    r = Encoder(5, 6)
    # l = Encoder (23, 24)
    while True:
        try:
            print(r.get_speed()/(2*math.pi))
            sleep(0.5)
    
        except KeyboardInterrupt:
            break
    
    
    pause()
