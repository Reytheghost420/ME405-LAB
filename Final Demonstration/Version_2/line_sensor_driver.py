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
    
     # Sensor positions for centroid
        self.positions = (-3, -2, -1, 0, 1, 2, 3)

        # -------------------------------------------------
        # IMPORTANT:
        # Set this based on your sensor behavior.
        # True  -> black gives LOWER ADC than white
        # False -> black gives HIGHER ADC than white
        # -------------------------------------------------
        self.INVERT = True # change based off of reading

        # Simple clamp range for normalization
        # You can tune these after testing raw values
        self.raw_min = 0
        self.raw_max = 65535

        # Detection thresholds
        self.line_threshold = 0.20         # minimum "blackness" to say line is visible
        self.checkpoint_threshold = 0.55   # how dark a sensor must be for checkpoint count
        self.checkpoint_count = 5          # how many sensors must see dark to count as CP

        # Optional debug counter
        self.print_count = 0

    # -------------------------------------------------
    # RAW SENSOR READ
    # -------------------------------------------------
    def read_raw(self):
        return [adc.read_u16() for adc in self.adc_pins]
    
    # -------------------------------------------------
    # NORMALIZE TO 0..1
    # 0 = white-ish, 1 = black-ish
    # -------------------------------------------------
    def read_weights(self):
        raw = self.read_raw()
        weights = []

        span = self.raw_max - self.raw_min
        if span <= 0:
            span = 1

        for val in raw:
            x = (val - self.raw_min) / span

            if x < 0.0:
                x = 0.0
            elif x > 1.0:
                x = 1.0

            # If black reads LOW, invert so black => larger weight
            if self.INVERT:
                x = 1.0 - x

            weights.append(x)

        return weights

    # -------------------------------------------------
    # CENTROID OF LINE
    # -------------------------------------------------
    def line_centroid(self, weights):
        total = 0.0
        weighted_sum = 0.0

        for pos, w in zip(self.positions, weights):
            weighted_sum += pos * w
            total += w

        if total < 1e-6:
            return None

        return weighted_sum / total
    
    # -------------------------------------------------
    # DOES THE ROBOT SEE ANY LINE?
    # Useful for garage line reacquisition
    # -------------------------------------------------
    def line_seen(self): # “Do at least a couple sensors currently see the line?”
        weights = self.read_weights()
        black_count = sum(1 for w in weights if w >= self.line_threshold)
        return black_count >= 2
    
    # -------------------------------------------------
    # CHECKPOINT / CROSS DETECTION
    # Counts when many sensors see dark at once
    # -------------------------------------------------
    def detect_checkpoint(self): # “Are enough sensors dark at once that this looks like a checkpoint/cross marker?”
        weights = self.read_weights()
        black_count = sum(1 for w in weights if w >= self.checkpoint_threshold)
        return black_count >= self.checkpoint_count

    # -------------------------------------------------
    # MAIN LINE FOLLOW CONTROLLER
    # -------------------------------------------------
    def update(self):
        now = utime.ticks_us()
        dt_us = utime.ticks_diff(now, self.last_t_us)
        self.last_t_us = now
        dt_s = max(dt_us / 1_000_000.0, 1e-6)

        weights = self.read_weights()
        error = self.line_centroid(weights)

        base_sp = float(self.base_sp.get())

        # If line is lost, go straight gently
        if error is None:
            self.e_int = 0.0
            self.left_sp.put(base_sp)
            self.right_sp.put(base_sp)
            return

        # Start simple: mostly proportional, tiny integral
        # Can later switch to reading kp_share/ki_share
        kp = 35.0
        ki = 0.0

        self.e_int += error * dt_s
        delta_sp = kp * error + ki * self.e_int

        # Limit steering correction
        max_delta = abs(base_sp)
        if delta_sp > max_delta:
            delta_sp = max_delta
        elif delta_sp < -max_delta:
            delta_sp = -max_delta

        # Positive error means line is to the right
        left_cmd = base_sp + delta_sp
        right_cmd = base_sp - delta_sp

        self.left_sp.put(left_cmd)
        self.right_sp.put(right_cmd)