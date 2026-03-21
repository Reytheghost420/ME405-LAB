from machine import Pin, ADC
import utime


class line_sensor_driver:
    def __init__(self, kp_share, ki_share, base_sp, left_sp, right_sp):
        self.kp_share = kp_share
        self.ki_share = ki_share
        self.base_sp  = base_sp
        self.left_sp  = left_sp
        self.right_sp = right_sp

        self.e_int     = 0.0
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

        # Sensor positions for centroid (-3 = leftmost, +3 = rightmost)
        self.positions = (-3, -2, -1, 0, 1, 2, 3)

        # True  -> black gives LOWER ADC than white
        # False -> black gives HIGHER ADC than white
        self.INVERT = False

        # Normalization range
        self.raw_min = 4500
        self.raw_max = 48000

        # Line detection threshold
        self.line_threshold = 0.20

        # Checkpoint detection
        self.checkpoint_threshold = 0.55
        self.checkpoint_count     = 3

        # Main thick line detection tuning
        self.main_line_dark_threshold   = 0.55
        self.main_line_min_dark_count   = 2
        self.main_line_center_tol       = 0.5
        self.main_line_min_total_weight = 1.0

        self.print_count = 0

    # -------------------------------------------------
    # RAW SENSOR READ
    # -------------------------------------------------
    def read_raw(self):
        return [adc.read_u16() for adc in self.adc_pins]

    # -------------------------------------------------
    # NORMALIZE TO 0..1  (0 = white, 1 = black)
    # -------------------------------------------------
    def read_weights(self):
        raw    = self.read_raw()
        weights = []
        span   = self.raw_max - self.raw_min
        if span <= 0:
            span = 1

        for val in raw:
            x = (val - self.raw_min) / span
            if x < 0.0:
                x = 0.0
            elif x > 1.0:
                x = 1.0
            if self.INVERT:
                x = 1.0 - x
            weights.append(x)

        return weights

    # -------------------------------------------------
    # CENTROID OF LINE
    # -------------------------------------------------
    def line_centroid(self, weights):
        total        = 0.0
        weighted_sum = 0.0

        for pos, w in zip(self.positions, weights):
            weighted_sum += pos * w
            total        += w

        if total < 1e-6:
            return None

        return weighted_sum / total

    # -------------------------------------------------
    # DOES THE ROBOT SEE ANY LINE?
    # -------------------------------------------------
    def line_seen(self):
        weights     = self.read_weights()
        black_count = sum(1 for w in weights if w >= self.line_threshold)
        return black_count >= 1

    # -------------------------------------------------
    # DOES THIS LOOK LIKE THE MAIN THICK LINE?
    # Used after garage exit to distinguish thick main
    # line from thin dashed or edge lines.
    # -------------------------------------------------
    def main_line_seen(self):
        weights  = self.read_weights()
        centroid = self.line_centroid(weights)

        if centroid is None:
            return False

        dark_count   = sum(1 for w in weights if w >= self.main_line_dark_threshold)
        total_weight = sum(weights)

        return (dark_count   >= self.main_line_min_dark_count and
                abs(centroid) <= self.main_line_center_tol    and
                total_weight  >= self.main_line_min_total_weight)

    # -------------------------------------------------
    # CHECKPOINT / CROSS DETECTION
    # -------------------------------------------------
    def detect_checkpoint(self):
        weights     = self.read_weights()
        black_count = sum(1 for w in weights if w >= self.checkpoint_threshold)
        return black_count >= self.checkpoint_count

    # -------------------------------------------------
    # MAIN LINE FOLLOW CONTROLLER
    # -------------------------------------------------
    def update(self):
        now   = utime.ticks_us()
        dt_us = utime.ticks_diff(now, self.last_t_us)
        self.last_t_us = now
        dt_s  = max(dt_us / 1_000_000.0, 1e-6)

        weights  = self.read_weights()
        error    = self.line_centroid(weights)
        base_sp  = float(self.base_sp.get())

        # Line lost -> go straight
        if error is None:
            self.e_int = 0.0
            self.left_sp.put(base_sp)
            self.right_sp.put(base_sp)
            return

        kp = 900
        ki = 50

        self.e_int += error * dt_s
        delta_sp    = kp * error + ki * self.e_int

        max_delta = abs(base_sp) * 0.8
        if delta_sp >  max_delta: delta_sp =  max_delta
        if delta_sp < -max_delta: delta_sp = -max_delta

        self.left_sp.put(base_sp  + delta_sp)
        self.right_sp.put(base_sp - delta_sp)

    # -------------------------------------------------
    # DEBUG
    # -------------------------------------------------
    def debug_raw(self):
        print("RAW:", self.read_raw())