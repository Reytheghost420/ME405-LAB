import serial
import time
import csv
import os
from datetime import datetime
import matplotlib.pyplot as plt


# -----------------------
# USER SETTINGS
# -----------------------

PORT = "COM3"
BAUD = 115200

KP = 0.05
KI = 0
SETPOINT = 8000
MOTOR = "l"          # "l" or "r"
USE_G = True         # True if you must press 'g' after selecting motor

LOG_DIR = "Collection Log"
os.makedirs(LOG_DIR, exist_ok=True)


# -----------------------
# Helper
# -----------------------

def timestamp():
    return datetime.now().strftime("%m_%d_%H_%M_%S")


# -----------------------
# Main
# -----------------------

with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
    time.sleep(2)  # allow board reset
    ser.reset_input_buffer()

    # --- Set gains ---
    ser.write(b"k")
    time.sleep(0.2)
    ser.write(f"{KP}\r".encode())
    time.sleep(0.2)
    ser.write(f"{KI}\r".encode())
    time.sleep(0.3)

    # --- Set setpoint ---
    ser.write(b"s")
    time.sleep(0.2)
    ser.write(f"{SETPOINT}\r".encode())
    time.sleep(0.3)

    # --- Start motor ---
    ser.write(MOTOR.encode())
    time.sleep(0.2)

    if USE_G:
        ser.write(b"g")

    print("Collecting data...")

    times = []
    speeds = []

    start_time = time.time()

    while time.time() - start_time < 15:  # collect max 15 sec
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        if "Waiting for go command" in line:
            break

        parts = line.split(",")

        if len(parts) == 2:
            try:
                t = float(parts[0])
                v = float(parts[1])
                times.append(t)
                speeds.append(v)
            except:
                pass

    if len(times) == 0:
        print("No data captured.")
        exit()

    stamp = timestamp()
    csv_file = os.path.join(LOG_DIR, f"{stamp}.csv")
    png_file = os.path.join(LOG_DIR, f"{stamp}.png")

    # --- Save CSV ---
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "speed_cps"])
        for t, v in zip(times, speeds):
            writer.writerow([t, v])

    # --- Plot ---
    plt.figure()
    plt.plot(times, speeds)
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (cps)")
    plt.title(f"Step Response (sp={SETPOINT}, Kp={KP}, Ki={KI})")
    plt.grid(True)
    plt.savefig(png_file)
    plt.close()

    from pathlib import Path
# ... earlier LOG_DIR = "Collection Log" ...
LOG_DIR = Path(LOG_DIR)        # convert to Path

# when saving:
csv_file = LOG_DIR / f"{stamp}.csv"
png_file = LOG_DIR / f"{stamp}.png"

# Print absolute path so you know where to look
print("Saved:", csv_file.resolve())
print("Saved:", png_file.resolve())
