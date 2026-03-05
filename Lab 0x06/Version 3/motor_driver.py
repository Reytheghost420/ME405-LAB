from pyb import Pin, Timer 

class motor_driver:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    
    def __init__(self, PWM, DIR, nSLP, chan, tim_num, freq=20000):
        '''Initializes a Motor object'''
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)

        self.tim = Timer(tim_num, freq=freq)
        self.PWM_chan = self.tim.channel(chan, pin=PWM, mode=Timer.PWM, pulse_width_percent=0)
    
    def set_effort(self, effort: float):
        '''Sets the present effort requested from the motor based on an input value
           between -100 and 100'''
        if 100>= effort >= 0:
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(effort)
        elif -100 <= effort < 0:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(-effort)
        pass
            
    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.nSLP_pin.high()
        self.PWM_chan.pulse_width_percent(0)
        pass
            
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP_pin.low()
        effort = 0
        self.PWM_chan.pulse_width_percent(effort)