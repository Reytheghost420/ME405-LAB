print("LOADED task_motor.py VERSION 3")

''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
'''
from motor_driver import motor_driver
from encoder      import encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                 mot: motor_driver, enc: encoder,
                 goFlag: Share, dataValues: Queue, timeValues:Queue, 
                 kp_share: Share, ki_share:Share, sp_share, mode_share):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: motor_driver = mot        # A motor object
        
        self._enc: encoder      = enc        # An encoder object
        
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        self._sp_share = sp_share
        self._mode_share = mode_share
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data

        self._kp_share = kp_share
        self._ki_share = ki_share
        self._step_applied = False

        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                # print("Initializing motor task")
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():
                    self._kp = float(self._kp_share.get())
                    self._ki = float(self._ki_share.get())
                    # print("Starting motor loop")
                    self._enc.zero() # reset encoder position 
                    self._mot.enable() # enable motor driver 
                    self._mot.set_effort(0) 
                    
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._startTime = ticks_us()
                    
                    self._e_int = 0.0
                    self._step_applied = False   # step starts at 0
                    self._state = S2_RUN
                
            elif self._state == S2_RUN: # Closed-loop control state
                # print(f"Running motor loop, cycle {self._dataValues.num_in()}")
                
                # Run the encoder update algorithm and then capture the present
                # position of the encoder. You will eventually need to capture
                # the motor speed instead of position here.
                # --- Update encoder and compute velocity (counts/sec) ---
                self._enc.update()
                vel_cps = self._enc.get_velocity() * 1_000_000  # counts/sec (since get_velocity is counts/us)

# --- Time stamp (relative to start) ---
                now = ticks_us()
                t_rel = ticks_diff(now, self._startTime)
                mode = int(self._mode_share.get())
                if mode == 0:
                    ref = 0.0 if not self._step_applied else float (self._sp_share.get())
                    self._step_applied = True
                else:
                    ref = float(self._sp_share.get())

# dt in seconds (ticks_diff is in microseconds)
                dt_s = self._enc.dt / 1_000_000
                if dt_s <= 0:
                     dt_s = 1e-6


    # --- PI control (start with Ki=0) ---
                e = ref - vel_cps
                self._e_int += e * dt_s

                effort = self._kp * e + self._ki * self._e_int
               
               # Saturate to motor_driver’s expected range (usually -100..100)
                if effort > 100:
                    effort = 100
                elif effort < -100:
                    effort = -100

                self._mot.set_effort(effort)

    # Store samples every loop (or downsample if you want)
                if not self._dataValues.full():
                    self._timeValues.put(t_rel)
                    self._dataValues.put(vel_cps)
           

# --- Stop condition when buffer full ---
            if self._dataValues.full():
               self._mot.set_effort(0)
               self._mot.disable()
               self._goFlag.put(False)
               self._state = S1_WAIT

                    
            
            yield self._state