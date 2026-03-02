from motor_driver  import motor_driver
from encoder       import encoder
from task_motor    import task_motor
from task_user     import task_user
from task_line     import task_line
from task_share    import Share, Queue, show_all
from cotask        import Task, task_list
from gc            import collect      
from IMU_driver    import IMU_driver
from task_observer import task_observer
import pyb 
from pyb          import Pin

imu = IMU_driver()

# Build all driver objects first
leftMotor    = motor_driver(Pin.cpu.B7, Pin.cpu.C13, Pin.cpu.C14, 2, 4)  # TIM4
rightMotor   = motor_driver(Pin.cpu.A7, Pin.cpu.A6,  Pin.cpu.B6,  2, 3)  # TIM3
leftEncoder  = encoder(1, pyb.Pin('A8'), pyb.Pin('A9'))
rightEncoder = encoder(5, pyb.Pin('A0'), pyb.Pin('A1'))

# Build shares and queues
leftMotorGo   = Share("B",     name="Left Mot. Go Flag")
rightMotorGo  = Share("B",     name="Right Mot. Go Flag")
kp_share      = Share("f",     name="Kp")
ki_share      = Share("f",     name="Ki")
left_sp       = Share('f',     name="left_sp_share")
right_sp      = Share("f",     name="right_sp_share")
mode_share    = Share("i",     name="Mode")
base_sp       = Share("f",     name="Base Speed")
dataValues    = Queue("f", 30, name="Data Collection Buffer")
timeValues    = Queue("L", 30, name="Time Buffer")
uL_eff        = Share("f", name="uL_effort")
uR_eff        = Share("f", name="uR_effort")

xhat_s        = Share("f", name="xhat_s")
xhat_psi      = Share("f", name="xhat_psi")
xhat_omL      = Share("f", name="xhat_omL")
xhat_omR      = Share("f", name="xhat_omR")


# Build task class objects
leftMotorTask  = task_motor(leftMotor,  leftEncoder,
                            leftMotorGo, dataValues, timeValues,
                            kp_share, ki_share, left_sp, mode_share)

rightMotorTask = task_motor(rightMotor, rightEncoder,
                            rightMotorGo, dataValues, timeValues,
                            kp_share, ki_share, right_sp, mode_share)

userTask = task_user(leftMotorGo, rightMotorGo, dataValues, timeValues,
                     kp_share, ki_share, base_sp, left_sp, right_sp, mode_share, 
                     xhat_s, xhat_psi, xhat_omL, xhat_omR)

line_task = task_line(kp_share, ki_share, base_sp,
                      left_sp, right_sp,
                      leftMotorGo, rightMotorGo,
                      mode_share)

task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(line_task.run, name="Line Follow",
                      priority=2, period=20))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))
observerTask = task_observer(leftEncoder, rightEncoder, imu,
                             uL_eff, uR_eff,
                             xhat_s, xhat_psi, xhat_omL, xhat_omR,
                             Ts=0.05, r=0.03, Vsup=6.0, ENC_CPR=1440)
task_list.append(Task(observerTask.run, name="Observer",
                      priority=2, period=50))

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