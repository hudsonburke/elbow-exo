import serial
import csv
import time
import matplotlib.pyplot as plt
from pathlib import Path

PORT = "/dev/cu.usbmodem1101"   # change if needed
BAUD = 115200

csv_path = Path(input("Enter trial/session name: ").strip() + ".csv")
chart_path = Path(input("Enter chart file name: ").strip() + ".png")

columns = [
    "trialNumber", "startCount", "countAt90", "finalCount",
    "countsTo90", "returnCounts", "totalCounts",
    "outputDegreesTo90", "returnDegrees", "totalDegrees",
    "timeTo90_s", "returnTime_s", "totalTrialTime_s"
]

if not csv_path.exists():
    with open(csv_path, "w", newline="") as f:
        csv.writer(f).writerow(columns)

def update_chart():
    with open(csv_path, "r") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return

    trials = [int(r["trialNumber"]) for r in rows]
    counts = [abs(int(float(r["countsTo90"]))) for r in rows]

    plt.figure()
    plt.bar(trials, counts)
    plt.xlabel("Trial")
    plt.ylabel("Counts to 90°")
    plt.title("Motor Counts Needed to Reach 90°")
    plt.xticks(trials)
    plt.tight_layout()
    plt.savefig(chart_path, dpi=300)
    plt.close()

    print(f"Chart updated: {chart_path}")

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("Connected.")
print("Commands:")
print("r = save current position as 90°")
print("s = save current position as 180°/start")
print("g = run trial")
print("z = reset encoder")
print("1 = manual reverse")
print("2 = manual forward")
print("0 = stop / emergency stop")
print("q = quit")

while True:
    user_cmd = input("Send command: ").strip()

    if user_cmd == "q":
        ser.write(b"0")
        break

    if user_cmd not in ["r", "s", "g", "z", "1", "2", "0"]:
        print("Invalid command.")
        continue

    ser.write(user_cmd.encode())

    if user_cmd in ["1", "2"]:
        print("Motor moving. Send 0 to stop.")
        continue

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line:
            print(line)

        if line == "CSV_ROW:":
            data_line = ser.readline().decode(errors="ignore").strip()
            print(data_line)

            values = data_line.split(",")

            if len(values) == len(columns):
                with open(csv_path, "a", newline="") as f:
                    csv.writer(f).writerow(values)

                print(f"Saved to {csv_path}")
                update_chart()
            else:
                print("CSV row parse error.")
                print(f"Expected {len(columns)} values, got {len(values)}")

        if "TRIAL_COMPLETE" in line or "ERROR" in line or "EMERGENCY_STOP" in line:
            break

        if user_cmd in ["r", "s", "z", "0"]:
            time.sleep(0.5)
            break

ser.write(b"0")
ser.close()
print("Done.")