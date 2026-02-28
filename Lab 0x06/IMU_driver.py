import struct
import pyb


dev_addr = 0x28
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
        myI2C.mem_write(1,0x28,0x3D)

        return {
            "x": acc_x,
            "y": acc_y,
            "z": acc_z
        }
    
imu = IMU_driver()
while True:
    sensor_data = imu.read_sensors()
    print(sensor_data)