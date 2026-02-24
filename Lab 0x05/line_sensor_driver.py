from machine      import Pin, ADC
from encoder      import encoder
from task_share   import Share, Queue
import utime


class line_sensor_driver:
    def __init__(self, kp_share, ki_share, base_sp, left_sp, right_sp):
        self.kp_share = kp_share
        self.ki_share = ki_share
        self.base_sp = base_sp
        self.left_sp = left_sp
        self.right_sp = right_sp

        self.e_int = 0.0
        self.last_t_us = utime.ticks_us()
        self.adc_pins = [
            ADC(Pin("PA4")),
            ADC(Pin("PB0")),
            ADC(Pin("PC1")),
            ADC(Pin("PC0")),
            ADC(Pin("PC2")),
            ADC(Pin("PC3")),
            ADC(Pin("PC5")),
        ]
        self.scale = 1 / 40000  # adjust to max sensor value

    def read_sensors(self):
        values = [adc.read_u16() for adc in self.adc_pins]
        scaled = [v * self.scale for v in values]
        return scaled

    def line_centroid(self, weights):
        positions = (-3, -2, -1, 0, 1, 2, 3) # sensor positions

        total = 0.0
        weighted_sum = 0.0

        for x, w in zip(positions, weights):
            weighted_sum += x * w               #multiply positions times weights
            total += w

        if total < 1e-6:
            return None  # line not detected (or too weak)

        return weighted_sum / total

    def update(self):
        now = utime.ticks_us()
        dt_us = utime.ticks_diff(now, self.last_t_us)
        self.last_t_us = now
        dt_s = max(dt_us / 1_000_000.0, 1e-6)

        weights = self.read_sensors()
        error = self.line_centroid(weights)

        base_sp = float(self.base_sp.get())

        # If line lost: keep going straight
        if error is None:
            self.e_int = 0.0
            self.left_sp.put(base_sp)
            self.right_sp.put(base_sp)
            return

        kp = float(self.kp_share.get())
        ki = float(self.ki_share.get())

        self.e_int += error * dt_s
        delta_sp = kp * error + ki * self.e_int

        # Clamp delta so you don't command crazy differential speeds
        max_delta = abs(base_sp)  # simple choice; tune as needed
        if delta_sp >  max_delta: delta_sp =  max_delta
        if delta_sp < -max_delta: delta_sp = -max_delta

        left_sp  = base_sp + delta_sp
        right_sp = base_sp - delta_sp

        self.left_sp.put(left_sp)
        self.right_sp.put(right_sp)