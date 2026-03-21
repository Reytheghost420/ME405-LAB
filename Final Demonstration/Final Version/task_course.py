# task_course.py
# Full course FSM.

import micropython
import math
from utime import ticks_ms, ticks_diff
from task_share import Share
from line_sensor_driver import line_sensor_driver

# ---------------- STATES ----------------
S0_IDLE             = micropython.const(0)
S1_FAST_LINE        = micropython.const(1)
S2_ARC              = micropython.const(2)
S2B_STRAIGHT        = micropython.const(16)
S3_GARAGE_TURN      = micropython.const(3)
S4_GARAGE_APPROACH  = micropython.const(4)
S4B_WALL_STOP       = micropython.const(10)
S5_FIND_LINE        = micropython.const(5)
S5A_SEARCH_LEFT     = micropython.const(11)
S5C_STRAIGHTEN      = micropython.const(13)
S6_MAIN_LINE        = micropython.const(6)
S6A_CP2_PAUSE       = micropython.const(15)
S6B_CP2_TURN        = micropython.const(14)
S6C_SLALOM          = micropython.const(17)
S7_TURN_180         = micropython.const(7)
S7B_FIND_RETURN     = micropython.const(18)
S8_RETURN_LINE      = micropython.const(8)
S8B_RETURN_STRAIGHT = micropython.const(19)
S9_FINISHED         = micropython.const(9)


class task_course:

    def __init__(self,
                 line_kp, line_ki, base_sp,
                 left_sp: Share, right_sp: Share,
                 left_go: Share, right_go: Share,
                 mode_share: Share,
                 ultra,
                 xhat_s: Share, xhat_psi: Share,
                 xhat_omL: Share, xhat_omR: Share,
                 omL_meas: Share, omR_meas: Share,
                 leftMotor, rightMotor):

        self._line = line_sensor_driver(line_kp, line_ki, base_sp, left_sp, right_sp)

        self._left_sp  = left_sp
        self._right_sp = right_sp
        self._left_go  = left_go
        self._right_go = right_go
        self._mode     = mode_share
        self._base_sp  = base_sp
        self._ultra    = ultra

        self._xhat_s   = xhat_s
        self._xhat_psi = xhat_psi
        self._xhat_omL = xhat_omL
        self._xhat_omR = xhat_omR
        self._omL_meas = omL_meas
        self._omR_meas = omR_meas

        self._leftMotor  = leftMotor
        self._rightMotor = rightMotor

        self._state      = S0_IDLE
        self._last_state = -1

        self._cp_count        = 0
        self._cp_latched      = False
        self._last_cp_time_ms = 0
        self._cp2_done        = False
        self._was_on_line     = False

        self._dist_cm            = None
        self._timer_ms           = ticks_ms()
        self._turn_start_heading = 0.0
        self._turn_target        = 0.0
        self._segment_start_s    = 0.0

        # --- Tunable parameters ---
        self._fast_base_speed      = 1500.0
        self._main_base_speed      = 1100.0
        self._return_base_speed    = 300
        self._turn_speed           = 300

        self._garage_wall_cm       = 11.0
        self._heading_tol          = 0.08

        self._finish_pause_ms      = 500
        self._wall_pause_ms        = 1000
        self._cp2_pause_ms         = 1000
        self._cp_latch_ms          = 1000

        self._arc_ms               = 2200
        self._straight_ms          = 1500
        self._garage_exit_turn_rad = -0.90

        self._search_left_cmd_L    = -180
        self._search_left_cmd_R    =  180
        self._search_left_ms       =  250
        self._straighten_ms        =  400

        self._garage_turn_ms       =  1000
        self._cp2_turn_ms          = 1600
        self._turn_180_ms          = 1400
        self._turn_effort          =   25

        self._find_return_ms       =  600
        self._return_straight_ms   = 3000  # ms - drive straight after return line ends

    # ----------------------------------------------------------
    # HELPERS
    # ----------------------------------------------------------

    def _enter_state(self, new_state):
        self._state    = new_state
        self._timer_ms = ticks_ms()
        if self._state != self._last_state:
            self._last_state = self._state

    def _set_wheel_speeds(self, left, right):
        self._left_sp.put(float(left))
        self._right_sp.put(float(right))

    def _stop_motors(self):
        self._left_sp.put(0.0)
        self._right_sp.put(0.0)
        self._leftMotor.set_effort(0)
        self._rightMotor.set_effort(0)

    def _start_segment_distance(self):
        self._segment_start_s = float(self._xhat_s.get())

    def _segment_distance(self):
        return abs(float(self._xhat_s.get()) - self._segment_start_s)

    def _wrap_angle(self, ang):
        while ang >  math.pi: ang -= 2.0 * math.pi
        while ang < -math.pi: ang += 2.0 * math.pi
        return ang

    def _heading_error(self, target, current):
        return self._wrap_angle(target - current)

    def _capture_turn_target(self, delta_rad):
        self._turn_start_heading = float(self._xhat_psi.get())
        self._turn_target = self._wrap_angle(self._turn_start_heading + delta_rad)

    def _checkpoint_seen(self):
        try:
            return bool(self._line.detect_checkpoint())
        except AttributeError:
            return False

    def _update_checkpoint_counter(self):
        seen = self._checkpoint_seen()
        now  = ticks_ms()
        if seen and not self._cp_latched:
            if ticks_diff(now, self._last_cp_time_ms) >= self._cp_latch_ms:
                self._cp_count       += 1
                self._cp_latched      = True
                self._last_cp_time_ms = now
        elif not seen:
            self._cp_latched = False

    def _reset_course(self):
        self._cp_count           = 0
        self._cp_latched         = False
        self._last_cp_time_ms    = 0
        self._cp2_done           = False
        self._was_on_line        = False
        self._dist_cm            = None
        self._turn_start_heading = 0.0
        self._turn_target        = 0.0
        self._segment_start_s    = float(self._xhat_s.get())

    # ----------------------------------------------------------
    # RUN LOOP
    # ----------------------------------------------------------
    def run(self):

        while True:

            # ---------------- IDLE ----------------
            if self._state == S0_IDLE:
                self._left_go.put(False)
                self._right_go.put(False)
                self._stop_motors()

                if int(self._mode.get()) == 2:
                    self._reset_course()
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._base_sp.put(self._main_base_speed)
                    self._enter_state(S1_FAST_LINE)

            # ---------------- FAST LINE: CP0 -> CP1 ----------------
            elif self._state == S1_FAST_LINE:
                self._base_sp.put(self._fast_base_speed)
                self._line.update()
                self._update_checkpoint_counter()

                if self._cp_count >= 1:
                    self._base_sp.put(self._main_base_speed)
                    self._enter_state(S2_ARC)

            # ---------------- ARC: time-based line follow ----------------
            elif self._state == S2_ARC:
                self._base_sp.put(self._main_base_speed)
                self._line.update()

                elapsed = ticks_diff(ticks_ms(), self._timer_ms)
                if elapsed >= self._arc_ms:
                    self._enter_state(S2B_STRAIGHT)

            # ---------------- STRAIGHT: correct angle then approach garage ----------------
            elif self._state == S2B_STRAIGHT:
                self._left_go.put(True)
                self._right_go.put(True)
                self._set_wheel_speeds(860, 700)

                elapsed = ticks_diff(ticks_ms(), self._timer_ms)
                if elapsed >= self._straight_ms:
                    self._stop_motors()
                    self._left_go.put(False)
                    self._right_go.put(False)
                    self._enter_state(S3_GARAGE_TURN)

            # ---------------- GARAGE TURN: direct effort right turn ----------------
            elif self._state == S3_GARAGE_TURN:
                elapsed = ticks_diff(ticks_ms(), self._timer_ms)

                self._left_go.put(False)
                self._right_go.put(False)
                self._leftMotor.enable()
                self._rightMotor.enable()
                self._leftMotor.set_effort(self._turn_effort)
                self._rightMotor.set_effort(-self._turn_effort)

                if elapsed >= self._garage_turn_ms:
                    self._leftMotor.set_effort(0)
                    self._rightMotor.set_effort(0)
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._enter_state(S4_GARAGE_APPROACH)

            # ---------------- GARAGE APPROACH: drive toward wall ----------------
            elif self._state == S4_GARAGE_APPROACH:
                self._dist_cm = self._ultra.get_distance_cm()
                self._set_wheel_speeds(1350, 1270)

                if self._dist_cm is not None and 0 < self._dist_cm <= self._garage_wall_cm:
                    self._left_sp.put(0.0)
                    self._right_sp.put(0.0)
                    self._base_sp.put(0.0)
                    self._left_go.put(False)
                    self._right_go.put(False)
                    self._enter_state(S4B_WALL_STOP)

            # ---------------- WALL STOP ----------------
            elif self._state == S4B_WALL_STOP:
                self._left_sp.put(0.0)
                self._right_sp.put(0.0)
                self._base_sp.put(0.0)
                self._left_go.put(False)
                self._right_go.put(False)

                elapsed = ticks_diff(ticks_ms(), self._timer_ms)
                if elapsed >= self._wall_pause_ms:
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._capture_turn_target(self._garage_exit_turn_rad)
                    self._enter_state(S5_FIND_LINE)

            # ---------------- EXIT TURN: heading-based left turn ----------------
            elif self._state == S5_FIND_LINE:
                psi = float(self._xhat_psi.get())
                err = self._heading_error(self._turn_target, psi)

                if abs(err) > self._heading_tol:
                    self._set_wheel_speeds(-400, 650)
                else:
                    self._stop_motors()
                    self._enter_state(S5A_SEARCH_LEFT)

            # ---------------- SWEEP LEFT: find main thick line ----------------
            elif self._state == S5A_SEARCH_LEFT:
                elapsed = ticks_diff(ticks_ms(), self._timer_ms)
                self._set_wheel_speeds(self._search_left_cmd_L,
                                       self._search_left_cmd_R)

                if self._line.main_line_seen():
                    self._stop_motors()
                    self._base_sp.put(self._main_base_speed)
                    self._left_sp.put(self._main_base_speed)
                    self._right_sp.put(self._main_base_speed)
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._enter_state(S5C_STRAIGHTEN)

                elif elapsed >= self._search_left_ms:
                    self._enter_state(S5A_SEARCH_LEFT)

            # ---------------- STRAIGHTEN ----------------
            elif self._state == S5C_STRAIGHTEN:
                self._base_sp.put(self._main_base_speed)
                self._left_sp.put(self._main_base_speed)
                self._right_sp.put(self._main_base_speed)

                elapsed = ticks_diff(ticks_ms(), self._timer_ms)
                if elapsed >= self._straighten_ms:
                    self._enter_state(S6_MAIN_LINE)

            # ---------------- MAIN LINE: cross area -> CP2 tip ----------------
            elif self._state == S6_MAIN_LINE:
                self._base_sp.put(self._main_base_speed)
                self._line.update()
                self._update_checkpoint_counter()

                if self._cp_count == 3 and not self._cp2_done:
                    self._cp2_done = True
                    self._stop_motors()
                    self._enter_state(S6A_CP2_PAUSE)

            # ---------------- CP2 PAUSE ----------------
            elif self._state == S6A_CP2_PAUSE:
                self._stop_motors()
                elapsed = ticks_diff(ticks_ms(), self._timer_ms)

                if elapsed >= self._cp2_pause_ms:
                    self._enter_state(S6B_CP2_TURN)

            # ---------------- CP2 RIGHT TURN: left wheel only ----------------
            elif self._state == S6B_CP2_TURN:
                elapsed = ticks_diff(ticks_ms(), self._timer_ms)

                self._left_go.put(False)
                self._right_go.put(False)
                self._leftMotor.enable()
                self._rightMotor.enable()
                self._leftMotor.set_effort(self._turn_effort)
                self._rightMotor.set_effort(0)

                if elapsed >= self._cp2_turn_ms:
                    self._leftMotor.set_effort(0)
                    self._rightMotor.set_effort(0)
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._base_sp.put(self._main_base_speed)
                    self._was_on_line = False
                    self._enter_state(S6C_SLALOM)

            # ---------------- SLALOM: line follow until line disappears ----------------
            elif self._state == S6C_SLALOM:
                self._base_sp.put(300)
                self._line.update()

                if self._line.line_seen():
                    self._was_on_line = True

                if self._was_on_line and not self._line.line_seen():
                    self._stop_motors()
                    self._left_go.put(False)
                    self._right_go.put(False)
                    self._enter_state(S7_TURN_180)

            # ---------------- 180 TURN: hardcoded direct effort ----------------
            elif self._state == S7_TURN_180:
                elapsed = ticks_diff(ticks_ms(), self._timer_ms)

                self._left_go.put(False)
                self._right_go.put(False)
                self._leftMotor.enable()
                self._rightMotor.enable()
                self._leftMotor.set_effort(-self._turn_effort)
                self._rightMotor.set_effort(self._turn_effort)

                if elapsed >= self._turn_180_ms:
                    self._leftMotor.set_effort(0)
                    self._rightMotor.set_effort(0)
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._enter_state(S7B_FIND_RETURN)

            # ---------------- FIND RETURN LINE: nudge right then line follow ----------------
            elif self._state == S7B_FIND_RETURN:
                elapsed = ticks_diff(ticks_ms(), self._timer_ms)
                self._left_go.put(True)
                self._right_go.put(True)
                self._set_wheel_speeds(200, 800)

                if elapsed >= self._find_return_ms:
                    self._base_sp.put(self._return_base_speed)
                    self._was_on_line = False
                    self._enter_state(S8_RETURN_LINE)

            # ---------------- RETURN LINE: follow until line ends ----------------
            elif self._state == S8_RETURN_LINE:
                self._base_sp.put(self._return_base_speed)
                self._line.update()

                if self._line.line_seen():
                    self._was_on_line = True

                if self._was_on_line and not self._line.line_seen():
                    self._was_on_line = False
                    self._enter_state(S8B_RETURN_STRAIGHT)

            # ---------------- RETURN STRAIGHT: drive straight for 1 second ----------------
            elif self._state == S8B_RETURN_STRAIGHT:
                self._left_go.put(True)
                self._right_go.put(True)
                self._set_wheel_speeds(self._return_base_speed,
                                       self._return_base_speed)

                elapsed = ticks_diff(ticks_ms(), self._timer_ms)
                if elapsed >= self._return_straight_ms:
                    self._stop_motors()
                    self._enter_state(S9_FINISHED)

            # ---------------- FINISHED ----------------
            elif self._state == S9_FINISHED:
                self._stop_motors()

                if ticks_diff(ticks_ms(), self._timer_ms) >= self._finish_pause_ms:
                    self._left_go.put(False)
                    self._right_go.put(False)
                    self._mode.put(0)
                    self._enter_state(S0_IDLE)

            yield self._state