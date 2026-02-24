from line_sensor_driver import line_sensor_driver
from task_share import Share
from utime import ticks_ms, ticks_diff
import micropython

S0_WAIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_line:
    def __init__(self, kp_share, ki_share, base_sp, left_sp: Share, right_sp: Share, left_go: Share, right_go: Share, mode_share):

        self._kp = kp_share
        self._ki = ki_share
        self._base = base_sp
        self._left_sp = left_sp
        self._right_sp = right_sp
        self._left_go = left_go
        self._right_go = right_go
        self._mode = mode_share
        self._driver = line_sensor_driver(kp_share, ki_share, base_sp, left_sp, right_sp)

        self._state = S0_WAIT


    def run(self):

        while True:

            if self._state == S0_WAIT:

                # Only run line follow when mode == 1
                if int(self._mode.get()) == 1:
                    self._state = S1_RUN

            elif self._state == S1_RUN:
                if int(self._mode.get()) != 1:
                    self._state = S0_WAIT
                else:
                    # Update left/right setpoints from line sensor
                    self._driver.update()

                    # Make sure motors are on while line-following
                    self._left_go.put(True)
                    self._right_go.put(True)

            yield self._state