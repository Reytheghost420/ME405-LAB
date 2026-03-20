# task_observer.py
import micropython
import math
from utime import ticks_us, ticks_diff

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_observer:
    def __init__(self, encL, encR, imu,
                 uL_effort_share, uR_effort_share,
                 xhat_s, xhat_psi, xhat_omL, xhat_omR, sL, sR, psi_dot, omL_meas, omR_meas,
                 Ts=0.05, r=0.03, Vsup=6.0,
                 ENC_CPR=1437.09, log_enable_share=None):
        self._encL = encL
        self._encR = encR
        self._imu  = imu

        self._uL_effort_share = uL_effort_share
        self._uR_effort_share = uR_effort_share

        self._xhat_s   = xhat_s
        self._xhat_psi = xhat_psi
        self._xhat_omL = xhat_omL
        self._xhat_omR = xhat_omR

        self.Ts = Ts
        self.r  = r
        self.Vsup = Vsup
        self.ENC_CPR = ENC_CPR


        self._log_share = log_enable_share 
        self._log_start_us = 0
        self._prev_log_state = 0

        self._sL = sL
        self._sR = sR
        self._psi_dot = psi_dot

        self.omL_meas = omL_meas
        self.omR_meas = omR_meas

        # --- Discrete matrices (precomputed for Ts=0.05) ---
        self.Ad = [
          [0.16529889, 0.0,        0.0,        0.0],
          [0.0,        0.17377394, 0.0,        0.0],
          [0.0,        0.0,        0.27253179, 0.0],
          [0.0,        0.0,        0.0,        0.28650480],
        ]

        # Bd columns: [uL, uR, s, psi, OmL, OmR]
        self.Bd = [
          [0.0,        0.0,        0.83470111, 0.0,        0.000347792, 0.000347792],
          [0.0,        0.0,        0.0,        0.82622606,-0.004721292, 0.004721292],
          [1.62778403, 0.0,        0.0,        0.0,        0.44767274,  0.0],
          [0.0,        1.66037873, 0.0,        0.0,        0.0,         0.42809712],
        ]

        self._state = S0_INIT
        self._last_us = ticks_us()

        # xhat = [s, psi, OmL, OmR]
        self._x = [0.0, 0.0, 0.0, 0.0]

        # heading unwrap support
        self._psi_prev = None
        self._psi_unwrapped = 0.0

    def _counts_to_rad(self, counts):
        return (2.0 * math.pi) * (counts / float(self.ENC_CPR))
    
    def _logging_on(self):
        if self._log_share is None:
            return False
        try:
            return int(self._log_share.get()) == 1
        except:
            return False
    
    def _cps_to_rads(self, cps):
        return (2.0 * math.pi) * (cps / float(self.ENC_CPR))

    def _unwrap_heading(self, psi):
        # unwrap to avoid jumps at +/-pi
        if self._psi_prev is None:
            self._psi_prev = psi
            self._psi_unwrapped = psi
            return psi

        d = psi - self._psi_prev
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi

        self._psi_unwrapped += d
        self._psi_prev = psi
        return self._psi_unwrapped

    def run(self):
        while True:
            if self._state == S0_INIT:
                # put IMU into IMU fusion mode once
                try:
                    self._imu.set_mode(0x08)
                except:
                    pass
                self._last_us = ticks_us()

                if self._logging_on():
                 print("t_us,sL,sR,psi_meas,psi_dot_meas,xhat_s,xhat_psi,xhat_omL,xhat_omR,omL_meas,omR_meas,uL_eff,uR_eff")

                self._state = S1_RUN

            elif self._state == S1_RUN:
                now = ticks_us()
                _ = ticks_diff(now, self._last_us)
                self._last_us = now

                # --- Measurements y = [s, psi, OmL, OmR] ---
                # You MUST adapt these two calls to your encoder class API:
                # - get_position() should return counts
                # - get_velocity() returns counts/us in your motor task usage
                cL = self._encL.get_position()
                cR = self._encR.get_position()

                thL = self._counts_to_rad(cL)
                thR = self._counts_to_rad(cR)

                sL = self.r * thL
                sR = self.r * thR
                s  = 0.5 * (sL + sR)

                psi = self._imu.read_heading_rad()
                psi = self._unwrap_heading(psi)

                psi_dot = self._imu.read_yaw_rate_rads()
                
                vL_cps = self._encL.get_velocity() * 1_000_000
                vR_cps = self._encR.get_velocity() * 1_000_000
                omL = self._cps_to_rads(vL_cps)
                omR = self._cps_to_rads(vR_cps)
                omL_meas = omL 
                omR_meas = omR 


                # --- Inputs u: convert effort% to volts ---
                uL_eff = float(self._uL_effort_share.get())
                uR_eff = float(self._uR_effort_share.get())
                uL = (uL_eff / 100.0) * self.Vsup
                uR = (uR_eff / 100.0) * self.Vsup

                ustar = [uL, uR, s, psi, omL, omR]

                # --- x_{k+1} = Ad x_k + Bd u*_k ---
                xnext = [0.0, 0.0, 0.0, 0.0]
                for i in range(4):
                    acc = 0.0
                    # Ad*x
                    for j in range(4):
                        acc += self.Ad[i][j] * self._x[j]
                    # Bd*ustar
                    for j in range(6):
                        acc += self.Bd[i][j] * ustar[j]
                    xnext[i] = acc

                self._x = xnext

                log_state = int(self._log_share.get()) if self._log_share else 0

                    # If logging just turned ON, reset start time
                if log_state == 1 and self._prev_log_state == 0:
                    self._log_start_us = now

                self._prev_log_state = log_state

                if log_state == 1:
                    t_rel = ticks_diff(now, self._log_start_us)

                    uL_eff = float(self._uL_effort_share.get())
                    uR_eff = float(self._uR_effort_share.get())
                    print("{:d},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}".format(
                        t_rel,
                        sL, sR,
                        psi, psi_dot,
                        self._x[0], self._x[1], self._x[2], self._x[3],
                        omL_meas, omR_meas,
                        uL_eff, uR_eff))
                if self._logging_on():
                     now = ticks_us()
                     
                # publish estimates
                self._xhat_s.put(self._x[0])
                self._xhat_psi.put(self._x[1])
                self._xhat_omL.put(self._x[2])
                self._xhat_omR.put(self._x[3])

                self._sL.put(sL)
                self._sR.put(sR)
                self._psi_dot.put(psi_dot)
                self.omL_meas.put(omL_meas)
                self.omR_meas.put(omR_meas)

            yield self._state