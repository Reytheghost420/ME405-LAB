print("LOADED task_motor.py FINAL VERSION 7")

from motor_driver import motor_driver
from encoder      import encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.

    Two independent invert flags handle the Romi's mirrored motor/encoder layout:

        invert_effort  : flips the effort sent to the motor driver.
                         Use when the motor spins the wrong direction.

        invert_encoder : flips the velocity reading from the encoder.
                         Use when the encoder reports negative velocity
                         while the wheel is actually moving forward.

    Left motor  -> invert_effort=True,  invert_encoder=False
    Right motor -> invert_effort=False, invert_encoder=True
    '''

    def __init__(self,
                 mot: motor_driver, enc: encoder,
                 goFlag: Share, dataValues: Queue, timeValues: Queue,
                 kp_share: Share, ki_share: Share, sp_share, mode_share,
                 uL_effort_share, uR_effort_share,
                 invert_effort:  bool = False,
                 invert_encoder: bool = False):

        self._state: int        = S0_INIT

        self._mot               = mot
        self._enc               = enc
        self._goFlag            = goFlag
        self._dataValues        = dataValues
        self._timeValues        = timeValues

        self._sp_share          = sp_share
        self._mode_share        = mode_share

        self._startTime: int    = 0
        self._run_start: int    = 0

        self._kp_share          = kp_share
        self._ki_share          = ki_share
        self._step_applied      = False

        self._uL_effort_share   = uL_effort_share
        self._uR_effort_share   = uR_effort_share

        self._effort_sign  = -1 if invert_effort  else 1
        self._encoder_sign = -1 if invert_encoder else 1

        print("Motor Task instantiated (invert_effort={}, invert_encoder={})".format(
              invert_effort, invert_encoder))

    def run(self):

        while True:

            if self._state == S0_INIT:
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                if not self._goFlag.get():
                    self._mot.set_effort(0)

                if self._goFlag.get():
                    self._kp = float(self._kp_share.get())
                    self._ki = float(self._ki_share.get())

                    self._enc.zero()
                    self._mot.enable()
                    self._mot.set_effort(0)

                    self._startTime    = ticks_us()
                    self._run_start    = self._startTime
                    self._e_int        = 0.0
                    self._step_applied = False

                    self._state = S2_RUN

            elif self._state == S2_RUN:
                if not self._goFlag.get():
                    self._mot.set_effort(0)
                    self._mot.disable()
                    self._state = S1_WAIT
                    yield self._state
                    continue

                self._enc.update()

                # Apply encoder sign fix independently of effort sign
                vel_cps = self._encoder_sign * self._enc.get_velocity() * 1_000_000

                now   = ticks_us()
                t_rel = ticks_diff(now, self._startTime)
                mode  = int(self._mode_share.get())

                if mode == 0:
                    if not self._step_applied:
                        ref = 0.0
                        self._step_applied = True
                    else:
                        ref = float(self._sp_share.get())
                else:
                    ref = float(self._sp_share.get())

                self._kp = float(self._kp_share.get())
                self._ki = float(self._ki_share.get())

                dt_s = self._enc.dt / 1_000_000
                if dt_s <= 0:
                    dt_s = 1e-6

                # --- PI control ---
                e = ref - vel_cps
                self._e_int += e * dt_s
                effort = self._kp * e + self._ki * self._e_int

                # Saturate
                if effort > 100:
                    effort = 100
                elif effort < -100:
                    effort = -100

                # --- Soft-start clamp ---
                t_since_start = ticks_diff(now, self._run_start)
                startup_limit = 12 if self._uL_effort_share is not None else 20
                if t_since_start < 500000:
                    if effort > startup_limit:
                        effort = startup_limit
                    elif effort < -startup_limit:
                        effort = -startup_limit

                # Apply effort sign fix independently of encoder sign
                applied_effort = self._effort_sign * effort

                self._mot.set_effort(applied_effort)

                if self._uL_effort_share is not None:
                    self._uL_effort_share.put(float(applied_effort))
                if self._uR_effort_share is not None:
                    self._uR_effort_share.put(float(applied_effort))

                if mode == 0:
                    if not self._dataValues.full():
                        self._timeValues.put(t_rel)
                        self._dataValues.put(vel_cps)

                    if self._dataValues.full():
                        self._mot.set_effort(0)
                        self._mot.disable()
                        self._goFlag.put(False)
                        self._state = S1_WAIT

            yield self._state