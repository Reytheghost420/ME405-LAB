from machine import time_pulse_us
import utime
from pyb import Pin

class ultrasonic_driver:
    def __init__(self, trig=Pin.cpu.B1, echo=Pin.cpu.B5):
        self.trig = Pin(trig, Pin.OUT_PP, value=0)
        self.echo = Pin(echo, Pin.IN)

    def get_distance_cm(self):
        # 10 us trigger pulse
        self.trig.low()
        self.trig.high()
        utime.sleep_us(2)
        self.trig.low()
        
        # Measure echo pulse width in microseconds
        dt = time_pulse_us(self.echo, 1, 30_000)  # 30ms timeout
        if dt < 0:
            return None  # -1 timeout waiting for pulse, -2 timeout during pulse (varies by port)

        return (dt * 0.0343) / 2
    
if __name__ == "__main__":
    ultrasonic = ultrasonic_driver()
    while True:
        distance = ultrasonic.get_distance_cm()
        if distance is not None:
            print("Distance: {:.1f} cm".format(distance))
        else:
            print("Error: {}".format(distance))

        utime.sleep_ms(100)

        #Before using it in the garage state, run the standalone test at the bottom and verify: 
        # far away wall → larger number
        # closer wall/hand → smaller number
        # readings are reasonably stable
        # at about 7 cm it reads near your threshold
        # If the readings jump too much, you can later average 2–3 readings, but first just get the basic pulse fixed.