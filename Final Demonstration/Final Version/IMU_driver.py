# IMU_driver.py
# BNO055 IMU Driver (ME405 Romi)
#
# Provides:
#   - set_mode(mode)
#   - read_heading_rad()         -> yaw/heading in radians (unwrapped elsewhere if desired)
#   - read_yaw_rate_rads()       -> yaw rate in rad/s
#   - read_euler_deg() / read_euler_raw()
#   - read_gyro_dps() / read_gyro_raw()
#   - calibration status + read/write calibration offsets
#
# Notes:
#   - Euler angles are reported in 1/16 degree units.
#   - Gyro is reported in 1/16 deg/s units.

import struct
import math
import pyb


class IMU_driver:
    # Default I2C address for BNO055 when ADR pin is LOW
    DEV_ADDR = 0x28

    # Registers
    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_CALIB_STAT = 0x35

    REG_EULER_H_LSB = 0x1A  # 6 bytes: heading, roll, pitch (each int16)
    REG_GYRO_X_LSB  = 0x14  # 6 bytes: gx, gy, gz (each int16)

    REG_CALIB_DATA_START = 0x55  # 22 bytes of offset data

    # Operating modes (from BNO055 datasheet)
    CONFIG_MODE     = 0x00
    IMU_MODE        = 0x08  # "IMU" (acc+gyro fusion, no magnetometer)
    NDOF_MODE       = 0x0C  # full fusion

    # Power modes (optional; normal is usually fine)
    PWR_NORMAL      = 0x00

    def __init__(self, i2c_bus=1, baudrate=100_000, address=DEV_ADDR):
        """Create an IMU driver on the given I2C bus."""
        self.addr = address
        self.i2c = pyb.I2C(i2c_bus, pyb.I2C.MASTER, baudrate=baudrate)

        # Put device in normal power mode and IMU mode by default
        try:
            self.i2c.mem_write(self.PWR_NORMAL, self.addr, self.REG_PWR_MODE)
        except Exception:
            # Some setups don’t need this or may throw on first boot; safe to ignore
            pass

        # Default to IMU mode
        self.set_mode(self.IMU_MODE)

    # ------------- Basic register helpers -------------

    def set_mode(self, mode):
        """Set BNO055 operating mode (e.g., CONFIG_MODE, IMU_MODE, NDOF_MODE)."""
        # Datasheet recommends switching to CONFIG_MODE before changing mode
        self.i2c.mem_write(self.CONFIG_MODE, self.addr, self.REG_OPR_MODE)
        pyb.delay(25)
        self.i2c.mem_write(mode, self.addr, self.REG_OPR_MODE)
        pyb.delay(25)

    # ------------- Euler angles -------------

    def read_euler_raw(self):
        """Return raw Euler int16 values (heading, roll, pitch) in 1/16 degrees."""
        buf = bytearray(6)
        self.i2c.mem_read(buf, self.addr, self.REG_EULER_H_LSB)
        heading, roll, pitch = struct.unpack('<hhh', buf)
        return heading, roll, pitch

    def read_euler_deg(self):
        """Return Euler angles (heading, roll, pitch) in degrees."""
        h, r, p = self.read_euler_raw()
        return (h / 16.0, r / 16.0, p / 16.0)

    def read_heading_rad(self):
        """Return heading (yaw) in radians."""
        heading_raw, _, _ = self.read_euler_raw()
        heading_deg = heading_raw / 16.0
        return heading_deg * math.pi / 180.0

    # ------------- Gyro -------------

    def read_gyro_raw(self):
        """Return raw gyro int16 values (gx, gy, gz) in 1/16 deg/s."""
        buf = bytearray(6)
        self.i2c.mem_read(buf, self.addr, self.REG_GYRO_X_LSB)
        gx, gy, gz = struct.unpack('<hhh', buf)
        return gx, gy, gz

    def read_gyro_dps(self):
        """Return gyro (gx, gy, gz) in deg/s."""
        gx, gy, gz = self.read_gyro_raw()
        return (gx / 16.0, gy / 16.0, gz / 16.0)

    def read_yaw_rate_rads(self):
        """Return yaw rate (gyro z) in rad/s."""
        _, _, gz = self.read_gyro_raw()
        gz_dps = gz / 16.0
        return gz_dps * math.pi / 180.0

    # ------------- Calibration helpers -------------

    def read_calibration_status(self):
        """
        Return calibration status byte.
        Bits are packed as: SYS(7:6), GYR(5:4), ACC(3:2), MAG(1:0)
        Each field ranges 0..3 (3 means fully calibrated).
        """
        buf = bytearray(1)
        self.i2c.mem_read(buf, self.addr, self.REG_CALIB_STAT)
        status = buf[0]
        return status

    def is_fully_calibrated(self):
        """True if SYS, GYR, ACC, MAG are all 3."""
        s = self.read_calibration_status()
        sys = (s >> 6) & 0x03
        gyr = (s >> 4) & 0x03
        acc = (s >> 2) & 0x03
        mag = (s >> 0) & 0x03
        print(sys, gyr, acc, mag)
        return (sys == 3 and gyr == 3 and acc == 3 and mag == 3)

    def read_calibration_data(self):
        """Read and return 22 bytes of calibration offset data."""
        buf = bytearray(22)
        self.i2c.mem_read(buf, self.addr, self.REG_CALIB_DATA_START)
        return bytes(buf)

    def write_calibration_data(self, calib_bytes):
        """
        Write 22 bytes of calibration offset data.
        Typically you must be in CONFIG_MODE before writing offsets.
        """
        if not isinstance(calib_bytes, (bytes, bytearray)) or len(calib_bytes) != 22:
            raise ValueError("calib_bytes must be 22 bytes")

        # Recommended: switch to CONFIG_MODE before writing offsets
        self.set_mode(self.CONFIG_MODE)
        pyb.delay(25)

        self.i2c.mem_write(calib_bytes, self.addr, self.REG_CALIB_DATA_START)
        pyb.delay(25)

        # Return to IMU mode by default
        self.set_mode(self.IMU_MODE)