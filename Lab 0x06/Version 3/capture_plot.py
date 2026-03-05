# capture_plot.py
# Captures CSV printed by your Romi over USB serial and (optionally) sends UI commands
# to set Kp/Ki, setpoint, and start a test.
#
# IMPORTANT:
# - Only ONE program can open the COM port at a time (close PuTTY / VS Code serial monitor).
# - Set PORT correctly before running.

import serial
import time
import csv
import os
from datetime import datetime

# ---------------- USER SETTINGS ----------------
PORT = "COM3"          # Windows: COM3, COM5...  Mac/Linux: /dev/ttyACM0 or /dev/ttyUSB0
BAUD = 115200

OUTDIR = "./captures"

# Choose what to do:
#   "baseline"  -> stop motors, just record
#   "straight"  -> set Kp/Ki, set base setpoint, start line-follow (f)
#   "stop"      -> send stop only
TEST_MODE = "baseline"

DURATION_S = 10

# Controller gains to send through UI ('k' command)
KP = 0.01
KI = 0.01

# Base speed setpoint to send through UI ('s' command)
SETPOINT_CPS = 900

# Timing between command chunks (sec). Increase if UI misses inputs.
CMD_DELAY = 0.35

# Send 'x' after recording ends
STOP_AFTER = True

# CSV format expectations
EXPECTED_COLUMNS = 11
HEADER_PREFIX = "t_us,"
# ------------------------------------------------


def send_cmd(ser: serial.Serial, s: str):
    """Send bytes exactly as if typed in a serial terminal."""
    ser.write(s.encode())
    ser.flush()
    time.sleep(CMD_DELAY)


def is_data_row(line: str) -> bool:
    """Detect if a line looks like a numeric CSV row with EXPECTED_COLUMNS fields."""
    if line.count(",") != (EXPECTED_COLUMNS - 1):
        return False
    parts = line.split(",")
    if len(parts) != EXPECTED_COLUMNS:
        return False
    try:
        int(parts[0])      # t_us
        float(parts[1])    # sL
        return True
    except Exception:
        return False


def run_test_commands(ser: serial.Serial):
    """
    Send UI commands based on task_user.py:
      x = stop
      k = set Kp then Ki (multichar input)
      s = set base setpoint (multichar input)
      f = line follow start
    """
    # Always stop first for safety
    send_cmd(ser, "x")

    if TEST_MODE == "baseline":
        return

    if TEST_MODE == "stop":
        return

    if TEST_MODE == "straight":
        # Set gains
        send_cmd(ser, "k")
        send_cmd(ser, f"{KP}\r")   # Kp
        send_cmd(ser, f"{KI}\r")   # Ki

        # Set base setpoint
        send_cmd(ser, "s")
        send_cmd(ser, f"{SETPOINT_CPS}\r")

        # Start line-follow (or your “run both motors” mode tied to 'f')
        send_cmd(ser, "f")
        return

    raise ValueError(f"Unknown TEST_MODE: {TEST_MODE}")


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    outname = os.path.join(OUTDIR, f"{TEST_MODE}_{ts}.csv")

    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} @ {BAUD}")
    print("Waiting for CSV header OR first valid data row (no reset required)...")

    fields = None
    first_row = None

    # Wait for header OR first valid data row
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        if line.startswith(HEADER_PREFIX):
            fields = line.split(",")
            print("Header detected.")
            break

        if is_data_row(line):
            fields = [
                "t_us", "sL", "sR", "psi_meas", "psi_dot_meas",
                "xhat_s", "xhat_psi", "xhat_omL", "xhat_omR",
                "uL_eff", "uR_eff"
            ]
            first_row = [p.strip() for p in line.split(",")]
            print("No header detected; starting from first data row.")
            break

    print(f"Recording to: {outname}")

    # Start the chosen test by sending UI commands (optional)
    run_test_commands(ser)
    print(f"Commands sent for TEST_MODE='{TEST_MODE}'. Recording for {DURATION_S} seconds...")
    if TEST_MODE == "straight":
        print(f"  KP={KP}, KI={KI}, SETPOINT_CPS={SETPOINT_CPS}")

    t0 = time.time()
    rows_written = 0

    with open(outname, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fields)

        if first_row is not None:
            writer.writerow(first_row)
            rows_written += 1

        while (time.time() - t0) < DURATION_S:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if not is_data_row(line):
                continue

            writer.writerow([p.strip() for p in line.split(",")])
            rows_written += 1

    if STOP_AFTER:
        send_cmd(ser, "x")
        print("Sent stop command 'x'.")

    ser.close()
    print(f"Done. Wrote {rows_written} rows.")
    print("Saved:", outname)


if __name__ == "__main__":
    main()