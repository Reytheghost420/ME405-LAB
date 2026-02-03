import pyb
from time import ticks_us, ticks_diff   # Use to get dt value in update()


class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chB_pin):
        '''Initializes an Encoder object'''
    
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 1     # Amount of time between last two updates

        self.period = 0xFFFF

        #create timer in encoder mode
        self.tim = pyb.Timer(tim, period=self.period, prescaler=0)
        self.tim.channel(1, pin=chA_pin, mode=pyb.Timer.ENC_AB)
        self.tim.channel(2, pin=chB_pin, mode=pyb.Timer.ENC_AB)


        #initialize timing
        self.prev_time = ticks_us()
        self.prev_count = self.tim.counter()
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        

        #read current time and counter 
        curr_time = ticks_us()
        curr_count = self.tim.counter()

        #compute raw delta
        delta = curr_count - self.prev_count

        #handle over flow and under flow
        if delta > (self.period // 2):
            delta -= (self.period + 1)
        elif delta < -(self.period // 2):
            delta += (self.period + 1)

        #update stored value of delta 
        self.delta = delta 
        self.position -= delta 
        self.dt = ticks_diff(curr_time, self.prev_time)

        self.prev_count = curr_count
        self.prev_time = curr_time 
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        
        if self.dt > 0:
            return -self.delta / self.dt 
        else: 
            return 0
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        
        self.position = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()
