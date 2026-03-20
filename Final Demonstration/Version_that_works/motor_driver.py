from pyb import Pin, Timer

class motor_driver:

    def __init__(self, PWM, DIR, nSLP, chan, tim_num, invert=False, freq=20000):
        self._invert = invert
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.DIR_pin  = Pin(DIR,  mode=Pin.OUT_PP)

        self.tim = Timer(tim_num, freq=freq)
        self.PWM_chan = self.tim.channel(chan, pin=PWM,
                                         mode=Timer.PWM,
                                         pulse_width_percent=0)

        self.nSLP_pin.high()

    def set_effort(self, effort: float):
       # if self.side == 'left':
        #    effort = -effort
        if self._invert: 
            effort = -effort

        if effort >= 0:
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(min(100, effort))
        else:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(min(100, -effort))

    def disable(self):
        self.PWM_chan.pulse_width_percent(0)
        self.nSLP_pin.low()

    def enable(self):
        self.nSLP_pin.high()