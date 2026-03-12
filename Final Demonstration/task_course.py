# task_course.py
# High-level finite state machine for full course navigation.

import micropython
from utime import ticks_ms, ticks_diff
from task_share import Share
from line_sensor_driver import line_sensor_driver
from pyb import Pin

# ---------------- STATES ----------------
S0_IDLE        = micropython.const(0)
S1_LINE_FOLLOW = micropython.const(1)
S2_OBSTACLE    = micropython.const(2)
S3_TURN        = micropython.const(3)
S4_FINISHED    = micropython.const(4)


class task_course:

    def __init__(self,
                 line_kp, line_ki, base_sp,
                 left_sp: Share, right_sp: Share,
                 left_go: Share, right_go: Share,
                 mode_share: Share,
                 ultrasonic_driver,
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
        self._ultra = ultrasonic_driver(Pin.cpu.B1, Pin.cpu.B5)

        # --- Observer shares ---
        self._xhat_s = xhat_s
        self._xhat_psi = xhat_psi

        # --- Internal variables ---
        self._state = S0_IDLE
        self._turn_start_heading = 0.0
        self._turn_target = 0.0
        self._obstacle_threshold = 7   # 7 cm

        self._timer_ms = ticks_ms()


    # ----------------------------------------------------------
    # RUN LOOP (FSM)
    # ----------------------------------------------------------
    def run(self):

        while True:

            # ---------------- IDLE ----------------
            if self._state == S0_IDLE:
                self._left_go.put(False)
                self._right_go.put(False)

                # Start when mode == 2 (choose your own code)
                if int(self._mode.get()) == 2:
                    print("Course Task: IDLE state")
                    self._left_go.put(True)
                    self._right_go.put(True)
                    self._state = S1_LINE_FOLLOW


            # ---------------- LINE FOLLOW ----------------
            elif self._state == S1_LINE_FOLLOW:
                print("Line follow state")
                # Run line controller
                self._line.update()

                # Obstacle detection (distance in cm)
                print(type(self._ultra))
                dist = self._ultra.get_distance_cm()

                if dist is not None and dist > 0 and dist < self._obstacle_threshold:
                    self._state = S2_OBSTACLE


            # ---------------- OBSTACLE DETECTED ----------------
            elif self._state == S2_OBSTACLE:
                print("Obstacle detected! Distance (cm):", dist)
                # Stop forward motion
                self._left_sp.put(0)
                self._right_sp.put(0)

                # Capture heading
                self._turn_start_heading = self._xhat_psi.get()
                self._turn_target = self._turn_start_heading + 1.57  # ~90° left

                self._state = S3_TURN


            # ---------------- TURN STATE ----------------
            elif self._state == S3_TURN:
                print("turning")
                psi = self._xhat_psi.get()
                error = self._turn_target - psi

                # Simple proportional turn controller
                K_turn = 150

                effort = K_turn * error

                # Saturate
                if effort > 200:
                    effort = 200
                elif effort < -200:
                    effort = -200

                self._left_sp.put(-effort)
                self._right_sp.put(effort)

                # Finish turn
                if abs(error) < 0.05:
                    self._state = S1_LINE_FOLLOW


            # ---------------- FINISHED ----------------
            elif self._state == S4_FINISHED:
                print("Course Task: FINISHED state")
                self._left_sp.put(0)
                self._right_sp.put(0)
                self._left_go.put(False)
                self._right_go.put(False)

            yield self._state