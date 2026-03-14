# task_course.py
# High-level finite state machine for full course navigation.

import micropython
import math 
from utime import ticks_ms, ticks_diff
from task_share import Share
from line_sensor_driver import line_sensor_driver
from pyb import Pin

# ---------------- STATES ----------------
S0_IDLE                 = micropython.const(0)
S1_FAST_LINE            = micropython.const(1)
S2_GARAGE_ENTRY         = micropython.const(2)
S3_GARAGE_TURN          = micropython.const(3)
S4_GARAGE_APPROACH      = micropython.const(4)
S5_FIND_LINE            = micropython.const(5)
S6_MAIN_LINE            = micropython.const(6)
S7_TURN_180             = micropython.const(7) 
S8_RETURN_LINE          = micropython.const(8)
S9_FINISHED             = micropython.const(9)

class task_course:

    def __init__(self,
                 line_kp, line_ki, base_sp,
                 left_sp: Share, right_sp: Share,
                 left_go: Share, right_go: Share,
                 mode_share: Share,
                 ultra,
                 xhat_s: Share, xhat_psi: Share):

        # --- Line sensor driver ---
        self._line = line_sensor_driver(line_kp, line_ki,
                                        base_sp, left_sp, right_sp)

        # --- Motor control shares ---
        self._left_sp = left_sp
        self._right_sp = right_sp
        self._left_go = left_go
        self._right_go = right_go
        self._mode = mode_share
        self._base_sp = base_sp

        # --- Ultrasonic driver ---
        self._ultra = ultra

        # --- Observer shares ---
        self._xhat_s = xhat_s
        self._xhat_psi = xhat_psi

        # --- Internal variables ---
        self._state = S0_IDLE
        self._last_state = -1

        self._cp_count = 0
        self._cp_latched = False

        self._dist_cm = None
        self._timer_ms = ticks_ms()

        self._turn_start_heading = 0.0
        self._turn_target = 0.0
        self._segment_start_s = 0.0

        # --- Tunable parameters ---
        self._garage_entry_dist_m = 0.16     # forward after CP1 before turning
        self._garage_wall_cm = 7.0           # wall detect threshold
        self._garage_back_dist_m = 0.05      # optional backup after wall touch
        self._reacquire_turn_speed = 70.0    # cps
        self._garage_drive_speed = 80.0      # cps
        self._fast_base_speed = 110.0        # cps
        self._main_base_speed = 90.0         # cps
        self._return_base_speed = 85.0       # cps
        self._turn_speed = 120.0             # cps
        self._heading_tol = 0.08             # rad
        self._finish_pause_ms = 500

    # ----------------------------------------------------------
    # HELPERS
    # ----------------------------------------------------------
    def _enter_state(self, new_state): #Safely switches the robot to a new state and records when the state started.
        self._state = new_state
        self._timer_ms = ticks_ms()

        if self._state != self._last_state:
            print("Course state ->", self._state)
            self._last_state = self._state

    def _set_wheel_speeds(self, left, right): #Sets motor speeds through the shared variables.
        self._left_sp.put(left)
        self._right_sp.put(right)

    def _stop_motors(self): #Stops the robot.
        self._left_sp.put(0.0)
        self._right_sp.put(0.0)

    def _start_segment_distance(self): #Stores the robot's starting position for measuring distance.
        self._segment_start_s = float(self._xhat_s.get())

    def _segment_distance(self): #Returns how far the robot has moved since _start_segment_distance().
        return abs(float(self._xhat_s.get()) - self._segment_start_s)

    def _wrap_angle(self, ang): #Keeps angles inside the range:
        while ang > math.pi:
            ang -= 2.0 * math.pi
        while ang < -math.pi:
            ang += 2.0 * math.pi
        return ang

    def _heading_error(self, target, current): #Computes shortest rotation between two angles.
        return self._wrap_angle(target - current)

    def _capture_turn_target(self, delta_rad): #Sets a new heading target relative to the current heading.
        self._turn_start_heading = float(self._xhat_psi.get())
        self._turn_target = self._wrap_angle(self._turn_start_heading + delta_rad)

    def _checkpoint_seen(self): #Asks the line sensor driver: "are we on a checkpoint marker?"
        # Assumes your line sensor driver implements detect_checkpoint()
        try:
            return bool(self._line.detect_checkpoint())
        except AttributeError:
            return False

    def _update_checkpoint_counter(self): #Counts checkpoints once each time they are crossed.
        seen = self._checkpoint_seen()

        if seen and not self._cp_latched:
            self._cp_count += 1
            self._cp_latched = True
            print("Checkpoint detected:", self._cp_count)

        elif not seen:
            self._cp_latched = False

    def _line_seen(self): #Checks if the robot can see the line again.
        # Assumes your line sensor driver implements line_seen()
        try:
            return bool(self._line.line_seen())
        except AttributeError:
            return False
        
    # ----------------------------------------------------------
    # RUN LOOP (FSM)
    # ----------------------------------------------------------
    def run(self):

        while True:

            # ---------------- IDLE ----------------
            if self._state == S0_IDLE:
                self._left_go.put(False)
                self._right_go.put(False)
                self._stop_motors()

                # Start when mode == 2 (choose your own code)
                if int(self._mode.get()) == 2:
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._base_sp.put(self._fast_base_speed)
                    self._enter_state(S1_FAST_LINE)


            # ---------------- FAST LINE: CP0 -> CP1 ----------------
            elif self._state == S1_FAST_LINE:
                self._base_sp.put(self._fast_base_speed)
                self._line.update()
                self._update_checkpoint_counter()

                # CP1 reached -> enter garage routine
                if self._cp_count >= 1:
                    self._start_segment_distance()
                    self._enter_state(S2_GARAGE_ENTRY)
            
            # ---------------- DRIVE FORWARD SLIGHTLY AFTER CP1 ----------------
            elif self._state == S2_GARAGE_ENTRY:
                self._set_wheel_speeds(self._garage_drive_speed, self._garage_drive_speed)

                if self._segment_distance() >= self._garage_entry_dist_m:
                    self._capture_turn_target(-math.pi / 2.0)   # right turn
                    self._enter_state(S3_GARAGE_TURN)
            
            # ---------------- TURN RIGHT INTO GARAGE ----------------
            elif self._state == S3_GARAGE_TURN:
                psi = float(self._xhat_psi.get())
                err = self._heading_error(self._turn_target, psi)

                if abs(err) > self._heading_tol:
                    # right turn
                    self._set_wheel_speeds(self._turn_speed, -self._turn_speed)
                else:
                    self._enter_state(S4_GARAGE_APPROACH)

            # ---------------- DRIVE TOWARD WALL ----------------
            elif self._state == S4_GARAGE_APPROACH:
                self._dist_cm = self._ultra.get_distance_cm()
                self._set_wheel_speeds(self._garage_drive_speed, self._garage_drive_speed)

                if self._dist_cm is not None and self._dist_cm > 0 and self._dist_cm <= self._garage_wall_cm:
                    self._capture_turn_target(math.pi / 2.0)    # left turn to look for line
                    self._enter_state(S5_FIND_LINE)

            # ---------------- TURN LEFT UNTIL LINE IS FOUND ----------------
            elif self._state == S5_FIND_LINE:
                self._set_wheel_speeds(-self._reacquire_turn_speed,
                                        self._reacquire_turn_speed)

                if self._line_seen():
                    self._base_sp.put(self._main_base_speed)
                    self._enter_state(S6_MAIN_LINE)

            # ---------------- MAIN LINE: FROM GARAGE TO CP4 ----------------
            elif self._state == S6_MAIN_LINE:
                self._base_sp.put(self._main_base_speed)
                self._line.update()
                self._update_checkpoint_counter()

                # Expected checkpoint progression:
                # CP1 already counted on top straight
                # CP2 after garage / cross area
                # CP3 after slalom
                # CP4 before dashed return segment
                if self._cp_count >= 4:
                    self._capture_turn_target(math.pi)          # 180 turn
                    self._enter_state(S7_TURN_180)

            # ---------------- 180 DEGREE TURN AT CP4 ----------------
            elif self._state == S7_TURN_180:
                psi = float(self._xhat_psi.get())
                err = self._heading_error(self._turn_target, psi)

                if abs(err) > self._heading_tol:
                    self._set_wheel_speeds(-self._turn_speed, self._turn_speed)
                else:
                    self._base_sp.put(self._return_base_speed)
                    self._enter_state(S8_RETURN_LINE)

            # ---------------- RETURN TO FINISH / CP5 ----------------
            elif self._state == S8_RETURN_LINE:
                self._base_sp.put(self._return_base_speed)
                self._line.update()
                self._update_checkpoint_counter()

                # CP5 is effectively the 5th checkpoint encountered
                if self._cp_count >= 5:
                    self._stop_motors()
                    self._enter_state(S9_FINISHED)

            # ---------------- FINISHED ----------------
            elif self._state == S9_FINISHED:
                self._stop_motors()
                print("Course Task: FINISHED state")

                if ticks_diff(ticks_ms(), self._timer_ms) >= self._finish_pause_ms:
                    self._left_go.put(False)
                    self._right_go.put(False)

            yield self._state