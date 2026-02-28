import struct
import pyb


dev_addr = 0x28
OPR_MODE = 0x3D

# Fusion Modes
CONFIG_MODE = 0x00
IMU = 0b1000
COMPASS = 0b1001
M4G = 0b1010
NDOF_FMC_OFF = 0b1011
NDOF = 0b1100

x_lsb = 0x08 # accelerometer x least significant byte
x_msb = 0x09 # accelerometer x most significant byte
y_lsb = 0x0A # accelerometer y least significant byte
y_msb = 0x0B # accelerometer y most significant byte
z_lsb = 0x0C # accelerometer z least significant byte
z_msb = 0x0D # accelerometer z most significant byte

class IMU_driver:
    def __init__(self):
        self.i2c = pyb.I2C(1, pyb.I2C.MASTER, baudrate=100000)

    def read_sensors(self):
        # Placeholder for reading sensor data
        myI2C = self.i2c
        buf = bytearray(0 for _ in range(6))  # Buffer to hold 6 bytes of data
        myI2C.mem_read(buf, dev_addr, x_lsb)
        acc_x, acc_y, acc_z = struct.unpack('<hhh', buf)  # Unpack the bytes into three 16-bit integers
        myI2C.mem_write(1,dev_addr,OPR_MODE)

        return {
            "x": acc_x,
            "y": acc_y,
            "z": acc_z
        }
    
    # Code that I'm writing away from the robot
    # 2
    def change_operating_mode(self, value):
        myI2C = self.i2c
        myI2C.mem_write(value,dev_addr,OPR_MODE)            # Write the chosen operating modme to the OPR_MODE register

    # 3
    def read_calibration_status(self):
        myI2C = self.i2c
        calibration_status_byte = bytearray(1)  # Buffer to hold the calibration status byte
        myI2C.mem_read(calibration_status_byte, dev_addr, 0x35)  # Read calibration status register
        if calibration_status_byte & 0b11000000 == 0b11000000:  # Check if the system is fully calibrated (bits 7 and 6 are both 1)
            print("System is fully calibrated.")
            return calibration_status_byte[0]                   # Return the calibration status byte as an integer
        else:
            print("System is not fully calibrated.")
            return calibration_status_byte[0]
        
    # 4    
    def read_calibration_data(self):
        myI2C = self.i2c
        calibration_data = bytearray(22)  # Buffer to hold 22 bytes of calibration data
        myI2C.mem_read(calibration_data, dev_addr, 0x55)  # Read calibration data starting from register 0x55
        return [bin(byte) for byte in calibration_data] # Return calibration data as a list of binary strings
    
    # 5 
    def write_calibration_data(self, calibration_data):
        myI2C = self.i2c
        myI2C.mem_write(calibration_data, dev_addr, 0x55)  # Write calibration data starting from register 0x55

    # 6
    def read_euler_angles(self):
        myI2C = self.i2c
        euler_data = bytearray(6)  # Buffer to hold 6 bytes of Euler angle data
        myI2C.mem_read(euler_data, dev_addr, 0x1A)  # Read Euler angles starting from register 0x1A
        heading, roll, pitch = struct.unpack('<hhh', euler_data)  # Unpack the bytes into three 16-bit integers
        return {
            "heading": heading,     # Read heading angle
            "roll": roll,       # Read roll angle
            "pitch": pitch                              
        }

    # 7
    def read_angular_velocity(self):
        myI2C = self.i2c
        gyro_data = bytearray(6)  # Buffer to hold 6 bytes of gyroscope data
        myI2C.mem_read(gyro_data, dev_addr, 0x14) # Read gyroscope data starting from register 0x14
        gyro_x, gyro_y, gyro_z = struct.unpack('<hhh', gyro_data)  # Unpack the bytes into three 16-bit integers
        return {
            "gyro_x": gyro_x,       # Read gyroscope x-axis data
            "gyro_y": gyro_y,       # Read gyroscope y-axis data
            "gyro_z": gyro_z        # Read gyroscope z-axis data
        }
    
# Testing section
imu = IMU_driver()
while True:
    imu.change_operating_mode(IMU)
    sensor_data = imu.read_sensors()
    print(sensor_data)
