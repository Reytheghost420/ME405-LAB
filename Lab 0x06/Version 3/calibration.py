import IMU_driver
CAL_FILE = "calibration.txt"

def calibrate(imu, filename=CAL_FILE):
    imu.set_mode(0x0C)

    try:
        with open(filename, "rb") as f:
            calib = f.read()

        if len(calib) == 22:
            print("Calibration file found.")
            print("Writing coefficients to BNO055...")
            imu.write_calibration_data(calib)
            return

        else:
            print("Invalid calibration file length.")

    except OSError:
        print("No calibration file found.")

    print("Calibrate IMU manually now...")
    print("Move sensor until fully calibrated.")

    while not imu.is_fully_calibrated():
        pass

    print("Fully calibrated.")

    calib = imu.read_calibration_data()
    
    with open(filename, "wb") as f:
        f.write(calib)

    print("Calibration saved to file.")