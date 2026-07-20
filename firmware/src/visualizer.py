#!/usr/bin/env python3

# Keyboard controls inside the plot window:
#   n          send selected PWM, move to final angle, then save CSV
#   up/+       increase selected trial PWM by 5
#   down/-     decrease selected trial PWM by 5
#   l          start/stop manual CSV logging
#   0-9        send selected target angle
#   x          start/stop sinusoidal trajectory in C++ code
#   left       manual motor reverse
#   right      manual motor forward
#   p/space    pause both motors, same as sending 's' in C++ code
#   r          recalibrate/zero joint angle
#   m          print C++ menu
#   c          clear Python-side stored telemetry
#   g          reset the graphs
#   q          quit visualizer

import csv
import re
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial


# =====================================================
# Configuration
# =====================================================

SERIAL_PORT = "COM8"
BAUD_RATE = 230400

# Edit this value to choose the PWM used by the next N trial.
DEFAULT_TRIAL_PWM = 150
PWM_STEP = 5

# Smaller overall window
WINDOW_SIZE = (15, 11.5)

# Slightly smaller 3D visualizer panel
ARM_PANEL_WIDTH = 0.95
GRAPH_PANEL_WIDTH = 1.0

HISTORY_POINTS = 1000
ANIMATION_INTERVAL_MS = 35

SHOULDER_TO_UPPER_IMU = 1.0
UPPER_IMU_TO_ELBOW = 5.25
ELBOW_TO_FOREARM_IMU = 3.0
FOREARM_IMU_TO_HAND = 1.0

IMU_SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

UPPER_ARM_DIRECTION_SIGN = -1.0
FOREARM_DIRECTION_SIGN = -1.0

PRINT_RAW_SERIAL = False

# CSV files are stored in this folder beside the Python program.
DATA_DIRECTORY = Path(__file__).resolve().parent / "system_id_data"

# Columns intentionally match the MATLAB system-identification script.
CSV_FIELDNAMES = [
    "computer_time",
    "time_s",
    "theta_deg",
    "m1_counts",
    "m2_counts",
    "pwm",
    "u_cmd",
    "mode",
    "trial_id",
]


# =====================================================
# Quaternion math
# =====================================================

def multiply_quaternions(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


DISPLAY_OFFSET_BASE = np.array([
    -0.7071067811865476,
    0.0,
    0.7071067811865476,
    0.0,
])

DISPLAY_OFFSET_EXTRA = np.array([
    -0.7071067811865476,
    0.0,
    0.0,
    0.7071067811865476,
])

DISPLAY_ROTATION = multiply_quaternions(
    DISPLAY_OFFSET_EXTRA,
    DISPLAY_OFFSET_BASE
)


def normalize_quaternion(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)

    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])

    return q / norm


def conjugate_quaternion(q):
    q = normalize_quaternion(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def rotate_vector_by_quaternion(vector, quaternion):
    quaternion = normalize_quaternion(quaternion)

    vector_quaternion = np.array([
        0.0,
        vector[0],
        vector[1],
        vector[2],
    ])

    rotated = multiply_quaternions(
        multiply_quaternions(quaternion, vector_quaternion),
        conjugate_quaternion(quaternion)
    )

    return rotated[1:]


def normalize_vector(vector):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)

    if norm < 1e-12:
        return np.array([0.0, 0.0, -1.0])

    return vector / norm


def quaternion_angle_degrees(quaternion):
    quaternion = normalize_quaternion(quaternion)
    w = abs(np.clip(quaternion[0], -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(w)))


# =====================================================
# Regex patterns for serial parsing
# =====================================================

FLOAT_PATTERN = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

QUATERNION_LINE_PATTERN = re.compile(
    rf"^\s*(qUpperZeroed|qForearmZeroed|qJointZeroed)\s*:\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN}),\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN})"
)

CURRENT_ANGLE_PATTERN = re.compile(rf"CurrentDeg\s*:\s*({FLOAT_PATTERN})")
TARGET_ANGLE_PATTERN = re.compile(rf"TargetDeg\s*:\s*({FLOAT_PATTERN})")
ERROR_ANGLE_PATTERN = re.compile(rf"ErrorDeg\s*:\s*({FLOAT_PATTERN})")

PWM_NORM_PATTERN = re.compile(rf"PWMNorm\s*:\s*({FLOAT_PATTERN})")
PWM_PATTERN = re.compile(r"PWM\s*:\s*(-?\d+)")
U_CMD_PATTERN = re.compile(rf"UCmd\s*:\s*({FLOAT_PATTERN})")

MODE_PATTERN = re.compile(r"Mode\s*:\s*([A-Za-z]+)")
COUNTS_PATTERN = re.compile(
    r"M1Counts\s*:\s*(-?\d+)\s*\|\s*M2Counts\s*:\s*(-?\d+)"
)


# =====================================================
# Runtime state
# =====================================================

serial_line_queue = deque()
stop_requested = threading.Event()
serial_connection = None

# CSV logging state
log_rows = []
logging_active = False
logging_label = ""
current_trial_id = 0
manual_log_number = 0
last_saved_csv = None


telemetry = {
    "upper_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
    "forearm_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
    "joint_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),

    "current_angle": 0.0,
    "target_angle": 0.0,
    "error_angle": 0.0,

    "pwm_norm": 0.0,
    "pwm": 0,
    "u_cmd": 0.0,

    "mode": "unknown",
    "motor_1_counts": 0,
    "motor_2_counts": 0,
    "board_time_s": 0.0,
    "trial_id": 0,

    "lines_read": 0,
    "quaternion_lines_read": 0,
}


# =====================================================
# Serial communication
# =====================================================

def open_serial_port(port, baud_rate):
    try:
        connection = serial.Serial(port, baud_rate, timeout=0.1)
        print(f"Opened serial port {port} @ {baud_rate}")
        return connection

    except Exception as error:
        print(f"Failed to open serial port {port}: {error}")
        return None


def send_serial_command(text, newline=True):
    if serial_connection is None or not serial_connection.is_open:
        print("Serial port is not open.")
        return

    payload = text + ("\n" if newline else "")

    try:
        serial_connection.write(payload.encode("utf-8"))
        printable_text = text.encode("unicode_escape").decode("ascii")
        print(f"Sent: {printable_text}")

    except Exception as error:
        print(f"Failed to send serial command: {error}")


def read_serial_lines(connection):
    try:
        while not stop_requested.is_set():
            try:
                raw_line = connection.readline()

            except Exception:
                break

            if not raw_line:
                continue

            line = raw_line.decode(errors="ignore").strip()

            if line:
                if PRINT_RAW_SERIAL:
                    print("RAW:", line)

                serial_line_queue.append(line)

    finally:
        try:
            connection.close()

        except Exception:
            pass


# =====================================================
# CSV logging
# =====================================================

def safe_filename_piece(text):
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", str(text)).strip("_")
    return cleaned or "recording"


def start_csv_logging(label, trial_id=0):
    global logging_active
    global logging_label
    global current_trial_id
    global log_rows

    if logging_active and log_rows:
        save_csv_logging("new_recording_started")

    DATA_DIRECTORY.mkdir(parents=True, exist_ok=True)

    logging_active = True
    logging_label = safe_filename_piece(label)
    current_trial_id = int(trial_id)
    log_rows = []

    print(
        f"CSV logging started: {logging_label} "
        f"(trial_id={current_trial_id})"
    )


def save_csv_logging(reason="completed"):
    global logging_active
    global logging_label
    global current_trial_id
    global log_rows
    global last_saved_csv

    if not logging_active:
        return None

    if not log_rows:
        print("CSV logging stopped, but no DATA samples were received.")
        logging_active = False
        logging_label = ""
        current_trial_id = 0
        return None

    DATA_DIRECTORY.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    reason_piece = safe_filename_piece(reason)
    label_piece = safe_filename_piece(logging_label)

    filename = f"{label_piece}_{timestamp}_{reason_piece}.csv"
    output_path = DATA_DIRECTORY / filename

    try:
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
            writer.writeheader()
            writer.writerows(log_rows)

        last_saved_csv = output_path
        print(f"Saved {len(log_rows)} samples to:")
        print(output_path)

    except OSError as error:
        print(f"Failed to save CSV: {error}")
        return None

    finally:
        logging_active = False
        logging_label = ""
        current_trial_id = 0
        log_rows = []

    return output_path


def parse_data_line(line):
    """Parse the compact C++ line:
    DATA,time_ms,theta_deg,m1_counts,m2_counts,pwm,u_cmd,mode,trial_id
    """
    global log_rows

    parts = [part.strip() for part in line.split(",")]

    if len(parts) != 9 or parts[0] != "DATA":
        return False

    try:
        time_s = float(parts[1]) / 1000.0
        theta_deg = float(parts[2])
        m1_counts = int(parts[3])
        m2_counts = int(parts[4])
        pwm = int(parts[5])
        u_cmd = float(parts[6])
        mode = parts[7]
        trial_id = int(parts[8])

    except ValueError:
        return False

    # Update live telemetry directly from the compact DATA record.
    telemetry["board_time_s"] = time_s
    telemetry["current_angle"] = theta_deg
    telemetry["motor_1_counts"] = m1_counts
    telemetry["motor_2_counts"] = m2_counts
    telemetry["pwm"] = pwm
    telemetry["u_cmd"] = u_cmd
    telemetry["pwm_norm"] = min(abs(pwm) / 255.0, 1.0)
    telemetry["mode"] = mode
    telemetry["trial_id"] = trial_id

    # Fallback: start logging if the EVENT line was missed but trial DATA arrives.
    if (
        not logging_active
        and trial_id > 0
        and mode in {
            "TrialWait",
            "TrialUp",
        }
    ):
        start_csv_logging(f"system_id_trial_{trial_id:03d}", trial_id)

    if logging_active:
        # For automatic trial logging, reject unrelated trial IDs.
        if current_trial_id > 0 and trial_id != current_trial_id:
            return True

        log_rows.append({
            "computer_time": datetime.now().isoformat(timespec="milliseconds"),
            "time_s": f"{time_s:.6f}",
            "theta_deg": f"{theta_deg:.6f}",
            "m1_counts": m1_counts,
            "m2_counts": m2_counts,
            "pwm": pwm,
            "u_cmd": f"{u_cmd:.6f}",
            "mode": mode,
            "trial_id": trial_id,
        })

    return True


def parse_event_line(line):
    """Use C++ EVENT lines to automatically begin and end trial logging."""
    parts = [part.strip() for part in line.split(",")]

    if len(parts) < 2 or parts[0] != "EVENT":
        return False

    event_name = parts[1]

    if event_name == "trial_start" and len(parts) >= 3:
        try:
            trial_id = int(parts[2])
        except ValueError:
            trial_id = 0

        start_csv_logging(f"system_id_trial_{trial_id:03d}", trial_id)
        return True

    if event_name in {"reached_final", "reached_90"}:
        save_csv_logging("completed_at_final_angle")
        return True

    if event_name == "timeout":
        save_csv_logging("timeout")
        return True

    if event_name == "emergency_stop":
        save_csv_logging("emergency_stop")
        return True

    return True


# =====================================================
# Serial parsing
# =====================================================

def parse_serial_line(line):
    telemetry["lines_read"] += 1

    if line.startswith("DATA,"):
        parse_data_line(line)
        return

    if line.startswith("EVENT,"):
        parse_event_line(line)
        return

    quaternion_match = QUATERNION_LINE_PATTERN.search(line)

    if quaternion_match:
        label = quaternion_match.group(1)

        quaternion = normalize_quaternion([
            float(quaternion_match.group(2)),
            float(quaternion_match.group(3)),
            float(quaternion_match.group(4)),
            float(quaternion_match.group(5)),
        ])

        if label == "qUpperZeroed":
            telemetry["upper_quaternion"] = quaternion

        elif label == "qForearmZeroed":
            telemetry["forearm_quaternion"] = quaternion

        elif label == "qJointZeroed":
            telemetry["joint_quaternion"] = quaternion

        telemetry["quaternion_lines_read"] += 1
        return

    current_angle_match = CURRENT_ANGLE_PATTERN.search(line)

    if current_angle_match:
        telemetry["current_angle"] = float(current_angle_match.group(1))

    target_angle_match = TARGET_ANGLE_PATTERN.search(line)

    if target_angle_match:
        telemetry["target_angle"] = float(target_angle_match.group(1))

    error_angle_match = ERROR_ANGLE_PATTERN.search(line)

    if error_angle_match:
        telemetry["error_angle"] = float(error_angle_match.group(1))

    pwm_norm_match = PWM_NORM_PATTERN.search(line)

    if pwm_norm_match:
        telemetry["pwm_norm"] = float(pwm_norm_match.group(1))

    pwm_match = PWM_PATTERN.search(line)

    if pwm_match:
        telemetry["pwm"] = int(pwm_match.group(1))

    u_cmd_match = U_CMD_PATTERN.search(line)

    if u_cmd_match:
        telemetry["u_cmd"] = float(u_cmd_match.group(1))

    mode_match = MODE_PATTERN.search(line)

    if mode_match:
        telemetry["mode"] = mode_match.group(1)

    counts_match = COUNTS_PATTERN.search(line)

    if counts_match:
        telemetry["motor_1_counts"] = int(counts_match.group(1))
        telemetry["motor_2_counts"] = int(counts_match.group(2))


def process_serial_queue():
    while serial_line_queue:
        parse_serial_line(serial_line_queue.popleft())


# =====================================================
# Arm position calculation
# =====================================================

def calculate_arm_positions():
    shoulder = np.array([0.0, 0.0, 0.0])
    base_axis = normalize_vector(IMU_SEGMENT_AXIS_LOCAL)

    upper_display_quaternion = normalize_quaternion(
        multiply_quaternions(
            DISPLAY_ROTATION,
            telemetry["upper_quaternion"]
        )
    )

    forearm_display_quaternion = normalize_quaternion(
        multiply_quaternions(
            DISPLAY_ROTATION,
            telemetry["forearm_quaternion"]
        )
    )

    upper_direction = rotate_vector_by_quaternion(
        base_axis * UPPER_ARM_DIRECTION_SIGN,
        upper_display_quaternion
    )

    forearm_direction = rotate_vector_by_quaternion(
        base_axis * FOREARM_DIRECTION_SIGN,
        forearm_display_quaternion
    )

    upper_direction = normalize_vector(upper_direction)
    forearm_direction = normalize_vector(forearm_direction)

    upper_imu = shoulder + SHOULDER_TO_UPPER_IMU * upper_direction

    elbow = shoulder + (
        SHOULDER_TO_UPPER_IMU + UPPER_IMU_TO_ELBOW
    ) * upper_direction

    forearm_imu = elbow + ELBOW_TO_FOREARM_IMU * forearm_direction

    hand = elbow + (
        ELBOW_TO_FOREARM_IMU + FOREARM_IMU_TO_HAND
    ) * forearm_direction

    return shoulder, upper_imu, elbow, forearm_imu, hand


def format_point(name, point):
    return f"{name}: ({point[0]: .3f}, {point[1]: .3f}, {point[2]: .3f})"


def reset_telemetry():
    telemetry.clear()

    telemetry.update({
        "upper_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "forearm_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "joint_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),

        "current_angle": 0.0,
        "target_angle": 0.0,
        "error_angle": 0.0,

        "pwm_norm": 0.0,
        "pwm": 0,
        "u_cmd": 0.0,

        "mode": "cleared",
        "motor_1_counts": 0,
        "motor_2_counts": 0,
        "board_time_s": 0.0,
        "trial_id": 0,

        "lines_read": 0,
        "quaternion_lines_read": 0,
    })


# =====================================================
# Main visualizer
# =====================================================

def main():
    global serial_connection

    serial_connection = open_serial_port(SERIAL_PORT, BAUD_RATE)

    if serial_connection is None:
        return

    serial_thread = threading.Thread(
        target=read_serial_lines,
        args=(serial_connection,),
        daemon=True
    )

    serial_thread.start()

    figure = plt.figure(figsize=WINDOW_SIZE)

    grid = figure.add_gridspec(
        4,
        2,
        width_ratios=[ARM_PANEL_WIDTH, GRAPH_PANEL_WIDTH],
        height_ratios=[1.0, 1.0, 1.0, 1.0],
        wspace=0.35,
        hspace=0.80
    )

    arm_axis = figure.add_subplot(grid[:, 0], projection="3d")
    angle_axis = figure.add_subplot(grid[0, 1])
    pwm_axis = figure.add_subplot(grid[1, 1])
    u_time_axis = figure.add_subplot(grid[2, 1])
    angle_u_axis = figure.add_subplot(grid[3, 1])

    figure.subplots_adjust(
        left=0.04,
        right=0.97,
        top=0.96,
        bottom=0.09
    )

    time_history = deque(maxlen=HISTORY_POINTS)
    angle_history = deque(maxlen=HISTORY_POINTS)
    pwm_norm_history = deque(maxlen=HISTORY_POINTS)

    u_history = deque(maxlen=HISTORY_POINTS)
    angle_for_u_history = deque(maxlen=HISTORY_POINTS)

    start_time = time.monotonic()
    selected_trial_pwm = max(0, min(255, int(DEFAULT_TRIAL_PWM)))

    total_arm_length = (
        SHOULDER_TO_UPPER_IMU
        + UPPER_IMU_TO_ELBOW
        + ELBOW_TO_FOREARM_IMU
        + FOREARM_IMU_TO_HAND
    )

    axis_limit = total_arm_length + 0.10

    arm_axis.set_xlim(-axis_limit, axis_limit)
    arm_axis.set_ylim(-axis_limit, axis_limit)
    arm_axis.set_zlim(-axis_limit, axis_limit)

    arm_axis.set_xlabel("X", fontsize=8)
    arm_axis.set_ylabel("Y", fontsize=8)
    arm_axis.set_zlabel("Z", fontsize=8)
    arm_axis.set_title("3D Arm Orientation From IMUs", fontsize=11, pad=8)
    arm_axis.tick_params(axis="both", which="major", labelsize=7)

    try:
        arm_axis.set_box_aspect([1, 1, 1])
    except Exception:
        pass

    shoulder, upper_imu, elbow, forearm_imu, hand = calculate_arm_positions()

    shoulder_dot, = arm_axis.plot(
        [shoulder[0]], [shoulder[1]], [shoulder[2]],
        "o", markersize=7
    )

    upper_imu_dot, = arm_axis.plot(
        [upper_imu[0]], [upper_imu[1]], [upper_imu[2]],
        "^", markersize=6
    )

    elbow_dot, = arm_axis.plot(
        [elbow[0]], [elbow[1]], [elbow[2]],
        "o", markersize=7
    )

    forearm_imu_dot, = arm_axis.plot(
        [forearm_imu[0]], [forearm_imu[1]], [forearm_imu[2]],
        "^", markersize=6
    )

    hand_dot, = arm_axis.plot(
        [hand[0]], [hand[1]], [hand[2]],
        "o", markersize=7
    )

    upper_arm_line, = arm_axis.plot(
        [shoulder[0], elbow[0]],
        [shoulder[1], elbow[1]],
        [shoulder[2], elbow[2]],
        linewidth=3
    )

    forearm_line, = arm_axis.plot(
        [elbow[0], hand[0]],
        [elbow[1], hand[1]],
        [elbow[2], hand[2]],
        linewidth=3
    )

    # ---------------------
    # Graph 1: theta(t) vs time
    # ---------------------

    angle_line, = angle_axis.plot(
        [],
        [],
        linewidth=2,
        label="θ(t)"
    )

    angle_axis.set_title("Output θ(t): Joint Angle vs Time", fontsize=10)
    angle_axis.set_xlabel("Time t (s)", fontsize=8)
    angle_axis.set_ylabel("Joint angle θ(t)", fontsize=8)
    angle_axis.grid(True, alpha=0.3)
    angle_axis.set_ylim(-180, 180)
    angle_axis.legend(loc="upper right", fontsize=7)

    # ---------------------
    # Graph 2: normalized PWM vs time
    # ---------------------

    pwm_norm_line, = pwm_axis.plot(
        [],
        [],
        linewidth=2,
        label="PWMNorm"
    )

    pwm_axis.set_title("Normalized Control Effort |u(t)|", fontsize=10)
    pwm_axis.set_xlabel("Time t (s)", fontsize=8)
    pwm_axis.set_ylabel("PWMNorm", fontsize=8)
    pwm_axis.grid(True, alpha=0.3)
    pwm_axis.set_ylim(-0.05, 1.05)
    pwm_axis.legend(loc="upper right", fontsize=7)

    # ---------------------
    # Graph 3: signed u(t) vs time
    # ---------------------

    u_time_line, = u_time_axis.plot(
        [],
        [],
        linewidth=2,
        label="u(t)"
    )

    u_time_axis.set_title("Input u(t): Signed PWM Command vs Time", fontsize=10)
    u_time_axis.set_xlabel("Time t (s)", fontsize=8)
    u_time_axis.set_ylabel("u(t) signed PWM", fontsize=8)
    u_time_axis.grid(True, alpha=0.3)
    u_time_axis.set_xlim(0.0, 1.0)
    u_time_axis.set_ylim(-260, 260)
    u_time_axis.legend(loc="upper right", fontsize=7)

    # ---------------------
    # Graph 4: theta(t) vs u(t)
    # ---------------------

    angle_u_line, = angle_u_axis.plot(
        [],
        [],
        linewidth=2,
        marker=".",
        markersize=3,
        label="θ(t) vs u(t)"
    )

    angle_u_axis.set_title("Input-Output Plot: θ(t) vs u(t)", fontsize=10)
    angle_u_axis.set_xlabel("Input u(t): signed PWM command", fontsize=8)
    angle_u_axis.set_ylabel("Output θ(t): joint angle", fontsize=8)
    angle_u_axis.grid(True, alpha=0.3)
    angle_u_axis.set_xlim(-260, 260)
    angle_u_axis.set_ylim(-5, 95)
    angle_u_axis.legend(loc="upper right", fontsize=7)

    # Smaller bottom status text
    status_text = figure.text(
        0.02,
        0.005,
        "",
        transform=figure.transFigure,
        verticalalignment="bottom",
        fontsize=7
    )

    def reset_plot_history():
        nonlocal start_time

        time_history.clear()
        angle_history.clear()
        pwm_norm_history.clear()
        u_history.clear()
        angle_for_u_history.clear()

        start_time = time.monotonic()

        angle_line.set_data([], [])
        angle_axis.set_xlim(0.0, 1.0)
        angle_axis.set_ylim(-180, 180)

        pwm_norm_line.set_data([], [])
        pwm_axis.set_xlim(0.0, 1.0)
        pwm_axis.set_ylim(-0.05, 1.05)

        u_time_line.set_data([], [])
        u_time_axis.set_xlim(0.0, 1.0)
        u_time_axis.set_ylim(-260, 260)

        angle_u_line.set_data([], [])
        angle_u_axis.set_xlim(-260, 260)
        angle_u_axis.set_ylim(-5, 95)

    def handle_key_press(event):
        global manual_log_number
        nonlocal selected_trial_pwm

        if event.key is None:
            return

        key = event.key.lower()

        print(f"Key pressed: {key}")

        if key == "n":
            reset_plot_history()

            # Send the exact selected PWM first. The C++ command format is:
            # v<PWM><Enter>, for example v150.
            send_serial_command(f"v{selected_trial_pwm}")
            send_serial_command("n")

            print(
                "One-way constant-PWM system-identification trial requested "
                f"at PWM {selected_trial_pwm}."
            )

        elif key in ["up", "+", "="]:
            selected_trial_pwm = min(
                255,
                selected_trial_pwm + PWM_STEP,
            )
            send_serial_command(f"v{selected_trial_pwm}")
            print(f"Selected trial PWM: {selected_trial_pwm}")

        elif key in ["down", "-"]:
            selected_trial_pwm = max(
                0,
                selected_trial_pwm - PWM_STEP,
            )
            send_serial_command(f"v{selected_trial_pwm}")
            print(f"Selected trial PWM: {selected_trial_pwm}")

        elif key == "l":
            if logging_active:
                save_csv_logging("manual_stop")
            else:
                manual_log_number += 1
                start_csv_logging(
                    f"manual_system_id_{manual_log_number:03d}",
                    trial_id=0,
                )

        elif key in [str(i) for i in range(10)]:
            send_serial_command(key)

        elif key == "left":
            send_serial_command("\x1b[D", newline=False)

        elif key == "right":
            send_serial_command("\x1b[C", newline=False)

        elif key == "x":
            send_serial_command("x")
            print("Trajectory command sent.")

        elif key == "r":
            send_serial_command("r")

        elif key in ["p", "space"]:
            send_serial_command("s")

        elif key == "m":
            send_serial_command("m")

        elif key == "c":
            reset_telemetry()
            reset_plot_history()
            print("Cleared Python-side telemetry.")

        elif key == "g":
            reset_plot_history()
            print("Reset graphs.")

        elif key == "q":
            if logging_active:
                save_csv_logging("quit")

            stop_requested.set()
            plt.close(figure)

    figure.canvas.mpl_connect("key_press_event", handle_key_press)

    def update_plot(_frame):
        process_serial_queue()

        shoulder, upper_imu, elbow, forearm_imu, hand = calculate_arm_positions()

        points = [shoulder, upper_imu, elbow, forearm_imu, hand]
        dots = [
            shoulder_dot,
            upper_imu_dot,
            elbow_dot,
            forearm_imu_dot,
            hand_dot,
        ]

        for dot, point in zip(dots, points):
            dot.set_data([point[0]], [point[1]])
            dot.set_3d_properties([point[2]])

        upper_arm_line.set_data(
            [shoulder[0], elbow[0]],
            [shoulder[1], elbow[1]]
        )

        upper_arm_line.set_3d_properties(
            [shoulder[2], elbow[2]]
        )

        forearm_line.set_data(
            [elbow[0], hand[0]],
            [elbow[1], hand[1]]
        )

        forearm_line.set_3d_properties(
            [elbow[2], hand[2]]
        )

        joint_angle = telemetry["current_angle"]

        if telemetry["quaternion_lines_read"] > 0 and abs(joint_angle) < 1e-9:
            joint_angle = quaternion_angle_degrees(
                telemetry["joint_quaternion"]
            )

        current_time = time.monotonic() - start_time

        time_history.append(current_time)
        angle_history.append(joint_angle)
        pwm_norm_history.append(telemetry["pwm_norm"])

        u_history.append(telemetry["u_cmd"])
        angle_for_u_history.append(joint_angle)

        # Graph 1: θ(t)
        angle_line.set_data(list(time_history), list(angle_history))

        if angle_history:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 1.0)

            angle_axis.set_xlim(left_time, right_time)

            min_angle = min(angle_history) - 5
            max_angle = max(angle_history) + 5

            if abs(max_angle - min_angle) < 1.0:
                min_angle -= 1.0
                max_angle += 1.0

            angle_axis.set_ylim(min_angle, max_angle)

        # Graph 2: PWMNorm
        pwm_norm_line.set_data(
            list(time_history),
            list(pwm_norm_history)
        )

        if pwm_norm_history:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 1.0)

            pwm_axis.set_xlim(left_time, right_time)
            pwm_axis.set_ylim(-0.05, 1.05)

        # Graph 3: u(t)
        u_time_line.set_data(
            list(time_history),
            list(u_history)
        )

        if u_history:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 1.0)

            u_time_axis.set_xlim(left_time, right_time)
            u_time_axis.set_ylim(-260, 260)

        # Graph 4: θ(t) vs u(t)
        angle_u_line.set_data(
            list(u_history),
            list(angle_for_u_history)
        )

        angle_u_axis.set_xlim(-260, 260)

        if angle_for_u_history:
            min_angle_u = min(angle_for_u_history) - 5.0
            max_angle_u = max(angle_for_u_history) + 5.0

            if abs(max_angle_u - min_angle_u) < 1.0:
                min_angle_u -= 1.0
                max_angle_u += 1.0

            angle_u_axis.set_ylim(min_angle_u, max_angle_u)

        status_lines = [
            (
                "Keys: n one-way system-ID trial | up/+ PWM+5 | down/- PWM-5 | "
                "l manual log | 0-9 target | x trajectory | "
                "left/right manual | p/space pause | "
                "r recalibrate | m menu | c clear | g reset | q quit"
            ),
            (
                f"Selected trial PWM: {selected_trial_pwm} | "
                f"CSV: {'RECORDING' if logging_active else 'idle'} | "
                f"Samples: {len(log_rows)} | "
                f"Trial ID: {telemetry['trial_id']} | "
                f"Board time: {telemetry['board_time_s']:.3f} s"
            ),
            (
                f"Target: {telemetry['target_angle']:.2f} deg | "
                f"θ(t): {joint_angle:.2f} deg | "
                f"Error: {telemetry['error_angle']:.2f} deg | "
                f"PWMNorm: {telemetry['pwm_norm']:.3f} | "
                f"PWM: {telemetry['pwm']} | "
                f"u(t)=UCmd: {telemetry['u_cmd']:.2f} | "
                f"Mode: {telemetry['mode']}"
            ),
            format_point("Shoulder", shoulder),
            format_point("Upper IMU", upper_imu),
            format_point("Elbow", elbow),
            format_point("Forearm IMU", forearm_imu),
            format_point("Hand", hand),
        ]

        status_text.set_text("\n".join(status_lines))

        return (
            *dots,
            upper_arm_line,
            forearm_line,
            angle_line,
            pwm_norm_line,
            u_time_line,
            angle_u_line,
            status_text,
        )

    arm_animation = animation.FuncAnimation(
        figure,
        update_plot,
        interval=ANIMATION_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    figure._arm_visualizer_animation = arm_animation

    try:
        plt.show()

    finally:
        if logging_active:
            save_csv_logging("window_closed")

        stop_requested.set()


if __name__ == "__main__":
    main()