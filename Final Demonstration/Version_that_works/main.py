import gc
gc.collect()
gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())

from motor_driver  import motor_driver
from encoder       import encoder
from task_motor    import task_motor
from task_user     import task_user
from task_line     import task_line
from task_share    import Share, Queue
from cotask        import Task, task_list
from gc            import collect      
from IMU_driver    import IMU_driver
from task_observer import task_observer
import pyb 
from pyb          import Pin
from calibration   import calibrate
from task_course    import task_course
from ultrasonic_driver import ultrasonic_driver
from gc            import collect
import gc
gc.collect()
gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())

imu = IMU_driver()

# Build all driver objects first
leftMotor    = motor_driver(Pin.cpu.B7, Pin.cpu.C10, Pin.cpu.C14, 2, 4)  # TIM4
rightMotor   = motor_driver(Pin.cpu.A7, Pin.cpu.A6,  Pin.cpu.B6,  2, 3)  # TIM3
leftEncoder  = encoder(1, pyb.Pin('A8'), pyb.Pin('A9'))
rightEncoder = encoder(5, pyb.Pin('A0'), pyb.Pin('A1'))
ultra        = ultrasonic_driver(Pin.cpu.B1, Pin.cpu.B5)

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
uL_effort_share    = Share("f", name="uL_effort")
uR_effort_share    = Share("f", name="uR_effort")

xhat_s        = Share("f", name="xhat_s")
xhat_psi      = Share("f", name="xhat_psi")
xhat_omL      = Share("f", name="xhat_omL")
xhat_omR      = Share("f", name="xhat_omR")
omL_meas      = Share("f", name="omL_meas")
omR_meas      = Share("f", name="omR_meas")

sL            = Share("f", name="Left Position")
sR            = Share("f", name="Right Position")

psi_dot       = Share("f", name="Yaw Rate")

log_enable    = Share("B", name="log_enable")
log_enable.put(0)

# Build task class objects
leftMotorTask  = task_motor(leftMotor,  leftEncoder,
                            leftMotorGo, dataValues, timeValues,
                            kp_share, ki_share, left_sp, mode_share, uL_effort_share, None, invert_effort=False,
                            invert_encoder=True)

rightMotorTask = task_motor(rightMotor, rightEncoder,
                            rightMotorGo, dataValues, timeValues,
                            kp_share, ki_share, right_sp, mode_share, None, uR_effort_share, invert_effort=False,
                            invert_encoder=False)

userTask = task_user(leftMotorGo, rightMotorGo, dataValues, timeValues,
                     kp_share, ki_share, base_sp, left_sp, right_sp, mode_share, 
                     xhat_s, xhat_psi, xhat_omL, xhat_omR, sL, sR, psi_dot, log_enable)

line_task = task_line(kp_share, ki_share, base_sp,
                      left_sp, right_sp,
                      leftMotorGo, rightMotorGo,
                      mode_share)


course_task = task_course(
                        kp_share, ki_share, base_sp,
                        left_sp, right_sp,
                        leftMotorGo, rightMotorGo,
                        mode_share,
                        ultra,
                        xhat_s, xhat_psi, xhat_omL, xhat_omR, omL_meas, omR_meas,
                        leftMotor, rightMotor)

task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))

observerTask = task_observer(leftEncoder, rightEncoder, imu,
                             uL_effort_share, uR_effort_share,
                             xhat_s, xhat_psi, xhat_omL, xhat_omR, sL, sR, psi_dot, omL_meas, omR_meas,
                             Ts=0.05, r=0.03, Vsup=6.0, ENC_CPR=1437.09,
                             log_enable_share=log_enable)

task_list.append(Task(observerTask.run, name="Observer",
                      priority=2, period=50))
task_list.append(Task(line_task.run, name="Line Follow",
                      priority=2, period=20))

task_list.append(Task(course_task.run,
                      name="Course Task",
                      priority=3,
                      period=20))


#Startup Values
#mode_share.put(2)        # 2 = run course task
kp_share.put(0.03)
ki_share.put(0.005)
mode_share.put(0)
base_sp.put(300.0)
left_sp.put(0.0)
right_sp.put(0.0)
leftMotorGo.put(False)
rightMotorGo.put(False)

leftMotorGo.put(False)
rightMotorGo.put(False)

# -------------------------------
# Run the garbage collector preemptively
collect()
#calibrate(imu)

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()   
    except KeyboardInterrupt:
        leftMotor.disable()
        rightMotor.disable()
        break