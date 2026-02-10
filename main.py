from motor_driver import motor_driver
from encoder      import encoder
from task_motor   import task_motor
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect      
import pyb 
from pyb          import Pin


# Build all driver objects first
leftMotor    = motor_driver(Pin.cpu.B7, Pin.cpu.C13, Pin.cpu.C14, 2, 4)  # TIM4
rightMotor   = motor_driver(Pin.cpu.A7, Pin.cpu.A6,  Pin.cpu.B6,  2, 3)  # TIM3
leftEncoder  = encoder(1, pyb.Pin('A8'), pyb.Pin('A9'))
rightEncoder = encoder(5, pyb.Pin('A0'), pyb.Pin('A1'))

# Build shares and queues
leftMotorGo   = Share("B",     name="Left Mot. Go Flag")
rightMotorGo  = Share("B",     name="Right Mot. Go Flag")
dataValues    = Queue("f", 30, name="Data Collection Buffer")
timeValues    = Queue("L", 30, name="Time Buffer")

# Build task class objects
leftMotorTask  = task_motor(leftMotor,  leftEncoder,
                            leftMotorGo, dataValues, timeValues)
rightMotorTask = task_motor(rightMotor, rightEncoder,
                            rightMotorGo, dataValues, timeValues)
userTask = task_user(leftMotorGo, rightMotorGo, dataValues, timeValues)

# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))

# Run the garbage collector preemptively
collect()

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
        
    except KeyboardInterrupt:
        print("Program Terminating")
        leftMotor.disable()
        rightMotor.disable()
        break

print("\n")
print(task_list)
print(show_all())