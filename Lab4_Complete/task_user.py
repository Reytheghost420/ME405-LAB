''' This file demonstrates an example UI task using a custom class with a
    run method implemented as a generator
'''
from pyb import USB_VCP
from task_share import Share
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_CMD  = micropython.const(1) # State 1 - wait for character input
S2_COL  = micropython.const(2) # State 2 - wait for data collection to end
S3_DIS  = micropython.const(3) # State 3 - display the collected data
S4_GET_KP = micropython.const(4)
S5_GET_KI = micropython.const(5)
S6_GET_SP = micropython.const(6)

HELP_MENU = (
"\r\n"
"\r\n"
"+----------------------------------------------------------+\r\n"
"| ME 405 Romi Tuning Interface Help Menu                   |\r\n"
"+----------------------------------------------------------+\r\n"
"|  h  | Print help menu                                    |\r\n"
"|  k  | Enter new gain values                              |\r\n"
"|  s  | Choose a new setpoint                              |\r\n"
"| L/R | Trigger step response and print results            |\r\n"
"+----------------------------------------------------------+\r\n"
)


UI_prompt = ">: "

def multichar_input(ser, out_share):
    char_buf = ""
    digits = set("0123456789")
    term = {"\r", "\n"}
    done = False

    while not done:
        if ser.any():
            char_in = ser.read(1).decode()

            if char_in in digits:
                ser.write(char_in)
                char_buf += char_in

            elif char_in == "." and "." not in char_buf:
                ser.write(char_in)
                char_buf += char_in

            elif char_in == "-" and len(char_buf) == 0:
                ser.write(char_in)
                char_buf += char_in

            elif char_in == "\x7f" and len(char_buf) > 0:
                ser.write(char_in)
                char_buf = char_buf[:-1]

            elif char_in in term:
                if len(char_buf) == 0:
                    ser.write("\r\nValue not changed\r\n")
                    done = True

                elif char_buf not in {"-", "."}:
                    value = float(char_buf)
                    out_share.put(value)
                    ser.write(f"\r\nValue set to {value}\r\n")
                    done = True

        yield  


class task_user:
    '''
    A class that represents a UI task. The task is responsible for reading user
    input over a serial port, parsing the input for single-character commands,
    and then manipulating shared variables to communicate with other tasks based
    on the user commands.
    '''

    def __init__(self, leftMotorGo, rightMotorGo, dataValues, timeValues, kp_share, ki_share, sp_share):
        '''
        Initializes a UI task object
        
        Args:
            leftMotorGo (Share):  A share object representing a boolean flag to
                                  start data collection on the left motor
            rightMotorGo (Share): A share object representing a boolean flag to
                                  start data collection on the right motor
            dataValues (Queue):   A queue object used to store collected encoder
                                  position values
            timeValues (Queue):   A queue object used to store the time stamps
                                  associated with the collected encoder data
        '''
        
        self._state: int          = S0_INIT      # The present state
        self._printed_menu = False
        
        self._leftMotorGo: Share  = leftMotorGo  # The "go" flag to start data
                                                 # collection from the left
                                                 # motor and encoder pair
        
        self._rightMotorGo: Share = rightMotorGo # The "go" flag to start data
                                                 # collection from the right
                                                 # motor and encoder pair
        
        self._ser: stream         = USB_VCP()    # A serial port object used to
                                                 # read character entry and to
                                                 # print output
        
        self._dataValues: Queue   = dataValues   # A reusable buffer for data
                                                 # collection
        self._active = None

        self._kp = kp_share
        self._ki = ki_share
        self._sp = sp_share
        self._subtask = None

        
        self._timeValues: Queue   = timeValues   # A reusable buffer for time
                                                 # stamping collected data
        
        self._ser.write("User Task object instantiated")

        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: 
                self._ser.write(HELP_MENU)
                self._ser.write(UI_prompt)
                self._printed_menu = True
                self._state = S1_CMD
                
            elif self._state == S1_CMD: # Wait for UI commands
                # Wait for at least one character in serial buffer
                if self._ser.any():
                    # Read the character and decode it into a string
                    inChar = self._ser.read(1).decode()
                    # If the character is an upper or lower case "l", start data
                    # collection on the left motor and if it is an "r", start
                    # data collection on the right motor
                    if inChar in {"h", "H"}:
                        self._ser.write(HELP_MENU)
                        self._ser.write(UI_prompt)

                    elif inChar in {"k", "K"}:
                        self._ser.write(UI_prompt)
                        self._ser.write("Enter proportional gain, Kp:")
                        self._subtask = multichar_input(self._ser, self._kp)
                        self._state = S4_GET_KP

                    elif inChar in {"s", "S"}:
                        self._ser.write("Enter setpoint:")
                        self._subtask = multichar_input(self._ser, self._sp)
                        self._state = S6_GET_SP

                    elif inChar in {"g", "G"}:
                        if self._active is None:
                            self._ser.write(
                                "\r\nSelect motor first (L or R)\r\n"
                                + UI_prompt
                            )

                    elif self._active == "L":
                        self._ser.write("\r\nStarting left motor...\r\n")
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(False)
                        self._state = S2_COL

                    elif self._active == "R":
                         self._ser.write("\r\nStarting right motor...\r\n")
                         self._rightMotorGo.put(True)
                         self._leftMotorGo.put(False)
                         self._state = S2_COL


                    if inChar in {"l", "L"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._active = "L"
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(False)
                        self._ser.write("Starting left motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._state = S2_COL
                    elif inChar in {"r", "R"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._active = "R"
                        self._rightMotorGo.put(True)
                        self._leftMotorGo.put(False)
                        self._ser.write("Starting right motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._state = S2_COL
                
            elif self._state == S2_COL:
                # While the data is collecting (in the motor task) block out the
                # UI and discard any character entry so that commands don't
                # queue up in the serial buffer
                if self._ser.any(): 
                    self._ser.read(1)
                
                # When both go flags are clear, the data collection must have
                # ended and it is time to print the collected data.
                done = ((self._active == "L" and not self._leftMotorGo.get()) or 
                       (self._active == "R" and not self._rightMotorGo.get()))
                
                if done: 
                    kp = self._kp.get()
                    ki = self._ki.get()
                    sp = self._sp.get()

                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write("Printing data...\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Setpoint: {:.3f} cps\r\n".format(sp))
                    self._ser.write("Kp: {:.6f} %/cps\r\n".format(kp))
                    self._ser.write("Ki: {:.6f} %/(cps*s)\r\n".format(ki))
                    self._ser.write("--------------------------------\r\n")
                    self._ser.write("Time [s],    Speed [cps]\r\n")
                    self._state = S3_DIS
                else:
                    self._state == S3_DIS
            
            elif self._state == S3_DIS:
                # While data remains in the buffer, print that data in a command
                # separated format. Otherwise, the data collection is finished.
                if self._dataValues.any():
                    t_us = self._timeValues.get()
                    vel = self._dataValues.get()

                    t_sec = t_us / 1_000_000  # convert microseconds to seconds

                    self._ser.write("{:.3f},    {:.3f}\r\n".format(t_sec, vel))



                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write(HELP_MENU)
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD

            elif self._state == S4_GET_KP:
                yield from self._subtask
                self._ser.write("Enter integral gain, Ki:")
                self._subtask = multichar_input(self._ser, self._ki)
                self._state = S5_GET_KI


            elif self._state == S5_GET_KI:
                yield from self._subtask
                self._ser.write(UI_prompt)
                self._state = S1_CMD


            elif self._state == S6_GET_SP:
                yield from multichar_input(self._ser, self._sp)
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            
            yield self._state