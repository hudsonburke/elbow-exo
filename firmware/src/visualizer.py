#!/usr/bin/env python3

# =====================================================
# Km Identification Visualizer + CSV Logger
#
# Keyboard controls inside the plot window:
#   n          start one fixed-time Km trial
#   v          select next worksheet PWM value on Arduino
#   + / -      increase/decrease TEST_PWM by 5
#   a / left   manual Motor 2 reverse/down
#   d / right  manual Motor 2 forward/up
#   s/p/e/space emergency stop
#   r          reset IMU and encoders
#   m          print Arduino menu
#   c          clear Python-side telemetry
#   g          reset graphs only
#   q          quit visualizer
#
# Expected Arduino DATA format:
#   DATA,time_ms,theta_deg,m1_counts,m2_counts,target_pwm,pwm,u_cmd,mode,trial_id
#
# Expected Arduino RESULT format:
#   RESULT,trial_id,pwm,u_cmd,test_time_s,c0,ct,delta_c,delta_l_m,cable_speed_mps,km_raw,km_deadband,early_stop
# =====================================================

import csv
import os
import re
import threading
import time
from collections import deque
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial


# =====================================================
# Configuration
# =====================================================

SERIAL_PORT = "COM8"
BAUD_RATE = 115200

OUTPUT_DIR = Path("km_identification_trials")

WINDOW_SIZE = (15, 11.5)
ARM_PANEL_WIDTH = 0.95
GRAPH_PANEL_WIDTH = 1.0

# Increased sample history for this test.
HISTORY_POINTS = 5000
ANIMATION_INTERVAL_MS = 35

SHOULDER_TO_UPPER_IMU = 1.0
UPPER_IMU_TO_ELBOW = 5.25
ELBOW_TO_FOREARM_IMU = 3.0
FOREARM_IMU_TO_HAND = 1.0

IMU_SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

UPPER_ARM_DIRECTION_SIGN = -1.0
FOREARM_DIRECTION_SIGN = -1.0

# Same constants used by the Arduino for extra Python-side calculations.
R_SPOOL_M = 0.01185
COUNTS_PER_REV = 17280.0
U_DEAD = 150

PRINT_RAW_SERIAL = False


# =====================================================
# Quaternion math from your working visualizer
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
# Regex patterns
# =====================================================

FLOAT_PATTERN = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?|nan|NaN|NAN"

QUATERNION_LINE_PATTERN = re.compile(
    rf"^\s*(qUpperZeroed|qForearmZeroed|qJointZeroed)\s*:\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN}),\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN})"
)

CURRENT_ANGLE_PATTERN = re.compile(rf"CurrentDeg\s*:\s*({FLOAT_PATTERN})")
TARGET_ANGLE_PATTERN = re.compile(rf"TargetDeg\s*:\s*({FLOAT_PATTERN})")
ERROR_ANGLE_PATTERN = re.compile(rf"ErrorDeg\s*:\s*({FLOAT_PATTERN})")

TARGET_PWM_PATTERN = re.compile(r"TargetPWM\s*:\s*(-?\d+)")
PWM_NORM_PATTERN = re.compile(rf"PWMNorm\s*:\s*({FLOAT_PATTERN})")
PWM_PATTERN = re.compile(r"(?:^|\|\s*)PWM\s*:\s*(-?\d+)")
U_CMD_PATTERN = re.compile(rf"UCmd\s*:\s*({FLOAT_PATTERN})")

MODE_PATTERN = re.compile(r"Mode\s*:\s*([A-Za-z]+)")
COUNTS_PATTERN = re.compile(
    r"M1Counts\s*:\s*(-?\d+)\s*\|\s*M2Counts\s*:\s*(-?\d+)"
)

DATA_PATTERN = re.compile(
    rf"^DATA,\s*(\d+),\s*({FLOAT_PATTERN}),\s*(-?\d+),\s*(-?\d+),"
    rf"\s*(-?\d+),\s*(-?\d+),\s*({FLOAT_PATTERN}),\s*([^,]+),\s*(\d+)"
)

RESULT_PATTERN = re.compile(r"^RESULT,")
EVENT_PATTERN = re.compile(r"^EVENT,")


def parse_float(text):
    if text.lower() == "nan":
        return float("nan")
    return float(text)


# =====================================================
# Runtime state
# =====================================================

serial_line_queue = deque()
stop_requested = threading.Event()
serial_connection = None

telemetry = {
    "upper_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
    "forearm_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
    "joint_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),

    "current_angle": 0.0,
    "target_angle": 0.0,
    "error_angle": 0.0,

    "target_pwm": 0,
    "pwm_norm": 0.0,
    "pwm": 0,
    "u_cmd": 0.0,

    "mode": "unknown",
    "trial_id": 0,
    "motor_1_counts": 0,
    "motor_2_counts": 0,

    "lines_read": 0,
    "data_lines_read": 0,
    "quaternion_lines_read": 0,
}

current_trial_id = None
current_trial_file = None
current_trial_writer = None
current_trial_path = None
samples_saved = 0
latest_event = ""
latest_result = ""

summary_file = None
summary_writer = None
events_file = None
events_writer = None


# =====================================================
# CSV helpers
# =====================================================

def ensure_output_files():
    global summary_file, summary_writer, events_file, events_writer

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    summary_path = OUTPUT_DIR / "km_summary.csv"
    events_path = OUTPUT_DIR / "km_events.csv"

    summary_new = not summary_path.exists()
    events_new = not events_path.exists()

    summary_file = open(summary_path, "a", newline="")
    summary_writer = csv.writer(summary_file)

    if summary_new:
        summary_writer.writerow([
            "computer_time",
            "trial_id",
            "pwm",
            "u_cmd",
            "test_time_s",
            "c0",
            "ct",
            "delta_c",
            "delta_l_m",
            "cable_speed_mps",
            "km_raw",
            "km_deadband",
            "early_stop",
        ])
        summary_file.flush()

    events_file = open(events_path, "a", newline="")
    events_writer = csv.writer(events_file)

    if events_new:
        events_writer.writerow(["computer_time", "raw_event_line"])
        events_file.flush()


def close_trial_file():
    global current_trial_file, current_trial_writer, current_trial_path

    if current_trial_file is not None:
        current_trial_file.flush()
        current_trial_file.close()

    current_trial_file = None
    current_trial_writer = None
    current_trial_path = None


def open_trial_file(trial_id, pwm):
    global current_trial_id, current_trial_file, current_trial_writer, current_trial_path
    global samples_saved

    close_trial_file()

    current_trial_id = int(trial_id)
    samples_saved = 0

    filename = f"trial_{current_trial_id:03d}_pwm_{int(pwm):03d}.csv"
    current_trial_path = OUTPUT_DIR / filename

    current_trial_file = open(current_trial_path, "w", newline="")
    current_trial_writer = csv.writer(current_trial_file)

    current_trial_writer.writerow([
        "trial_id",
        "time_s",
        "theta_deg",
        "m1_counts",
        "m2_counts",
        "delta_m2_counts_from_first_sample",
        "cable_displacement_m_from_first_sample",
        "target_pwm",
        "actual_pwm",
        "u_cmd",
        "mode",
    ])
    current_trial_file.flush()

    print(f"Opened trial CSV: {current_trial_path}")


first_m2_count_this_trial = None


def reset_first_count():
    global first_m2_count_this_trial
    first_m2_count_this_trial = None


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
# Serial parsing
# =====================================================

def parse_data_line(line):
    global samples_saved, first_m2_count_this_trial

    match = DATA_PATTERN.search(line)

    if not match:
        return False

    time_ms = int(match.group(1))
    theta_deg = parse_float(match.group(2))
    m1_counts = int(match.group(3))
    m2_counts = int(match.group(4))
    target_pwm = int(match.group(5))
    pwm = int(match.group(6))
    u_cmd = parse_float(match.group(7))
    mode = match.group(8).strip()
    trial_id = int(match.group(9))

    telemetry["current_angle"] = theta_deg
    telemetry["motor_1_counts"] = m1_counts
    telemetry["motor_2_counts"] = m2_counts
    telemetry["target_pwm"] = target_pwm
    telemetry["pwm"] = pwm
    telemetry["u_cmd"] = u_cmd
    telemetry["mode"] = mode
    telemetry["trial_id"] = trial_id
    telemetry["pwm_norm"] = abs(pwm) / 255.0 if pwm else 0.0
    telemetry["data_lines_read"] += 1

    if current_trial_writer is not None and trial_id == current_trial_id:
        if first_m2_count_this_trial is None:
            first_m2_count_this_trial = m2_counts

        delta_counts = m2_counts - first_m2_count_this_trial
        delta_phi = (2.0 * np.pi / COUNTS_PER_REV) * delta_counts
        cable_disp = R_SPOOL_M * delta_phi

        current_trial_writer.writerow([
            trial_id,
            f"{time_ms / 1000.0:.4f}",
            f"{theta_deg:.4f}",
            m1_counts,
            m2_counts,
            delta_counts,
            f"{cable_disp:.8f}",
            target_pwm,
            pwm,
            f"{u_cmd:.3f}",
            mode,
        ])

        samples_saved += 1

        if samples_saved % 25 == 0:
            current_trial_file.flush()

    return True


def parse_event_line(line):
    global latest_event

    if not EVENT_PATTERN.search(line):
        return False

    latest_event = line

    if events_writer is not None:
        events_writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), line])
        events_file.flush()

    parts = line.split(",")

    # EVENT,trial_start,1,pwm,150,u_cmd,150,test_time_ms,4000,c0,0
    if len(parts) >= 5 and parts[1] == "trial_start":
        try:
            trial_id = int(parts[2])
            pwm = parts[parts.index("pwm") + 1]
            open_trial_file(trial_id, pwm)
            reset_first_count()
        except Exception as error:
            print(f"Could not open trial file from event: {error}")

    if len(parts) >= 3 and parts[1] == "pwm_set":
        try:
            telemetry["target_pwm"] = int(parts[2])
        except Exception:
            pass

    if len(parts) >= 2 and parts[1] in ["stop", "trial_end"]:
        if current_trial_file is not None:
            current_trial_file.flush()

    return True


def parse_result_line(line):
    global latest_result

    if not RESULT_PATTERN.search(line):
        return False

    latest_result = line

    parts = line.split(",")

    try:
        result = {
            "trial_id": int(parts[1]),
            "pwm": int(parts[2]),
            "u_cmd": float(parts[3]),
            "test_time_s": float(parts[4]),
            "c0": int(parts[5]),
            "ct": int(parts[6]),
            "delta_c": int(parts[7]),
            "delta_l_m": float(parts[8]),
            "cable_speed_mps": float(parts[9]),
            "km_raw": parse_float(parts[10]),
            "km_deadband": parse_float(parts[11]),
            "early_stop": int(parts[12]),
        }

        summary_writer.writerow([
            time.strftime("%Y-%m-%d %H:%M:%S"),
            result["trial_id"],
            result["pwm"],
            result["u_cmd"],
            f"{result['test_time_s']:.4f}",
            result["c0"],
            result["ct"],
            result["delta_c"],
            f"{result['delta_l_m']:.8f}",
            f"{result['cable_speed_mps']:.8f}",
            result["km_raw"],
            result["km_deadband"],
            result["early_stop"],
        ])
        summary_file.flush()

        if current_trial_file is not None:
            current_trial_file.flush()

        print("Saved RESULT to km_summary.csv")

    except Exception as error:
        print(f"Could not parse RESULT line: {error}")
        print(line)

    return True


def parse_debug_and_quat_line(line):
    quaternion_match = QUATERNION_LINE_PATTERN.search(line)

    if quaternion_match:
        label = quaternion_match.group(1)

        quaternion = normalize_quaternion([
            parse_float(quaternion_match.group(2)),
            parse_float(quaternion_match.group(3)),
            parse_float(quaternion_match.group(4)),
            parse_float(quaternion_match.group(5)),
        ])

        if label == "qUpperZeroed":
            telemetry["upper_quaternion"] = quaternion

        elif label == "qForearmZeroed":
            telemetry["forearm_quaternion"] = quaternion

        elif label == "qJointZeroed":
            telemetry["joint_quaternion"] = quaternion

        telemetry["quaternion_lines_read"] += 1
        return True

    current_angle_match = CURRENT_ANGLE_PATTERN.search(line)

    if current_angle_match:
        telemetry["current_angle"] = parse_float(current_angle_match.group(1))

    target_angle_match = TARGET_ANGLE_PATTERN.search(line)

    if target_angle_match:
        telemetry["target_angle"] = parse_float(target_angle_match.group(1))

    error_angle_match = ERROR_ANGLE_PATTERN.search(line)

    if error_angle_match:
        telemetry["error_angle"] = parse_float(error_angle_match.group(1))

    target_pwm_match = TARGET_PWM_PATTERN.search(line)

    if target_pwm_match:
        telemetry["target_pwm"] = int(target_pwm_match.group(1))

    pwm_norm_match = PWM_NORM_PATTERN.search(line)

    if pwm_norm_match:
        telemetry["pwm_norm"] = parse_float(pwm_norm_match.group(1))

    pwm_match = PWM_PATTERN.search(line)

    if pwm_match:
        telemetry["pwm"] = int(pwm_match.group(1))

    u_cmd_match = U_CMD_PATTERN.search(line)

    if u_cmd_match:
        telemetry["u_cmd"] = parse_float(u_cmd_match.group(1))

    mode_match = MODE_PATTERN.search(line)

    if mode_match:
        telemetry["mode"] = mode_match.group(1)

    counts_match = COUNTS_PATTERN.search(line)

    if counts_match:
        telemetry["motor_1_counts"] = int(counts_match.group(1))
        telemetry["motor_2_counts"] = int(counts_match.group(2))

    return False


def parse_serial_line(line):
    telemetry["lines_read"] += 1

    if parse_data_line(line):
        return

    if parse_event_line(line):
        return

    if parse_result_line(line):
        return

    parse_debug_and_quat_line(line)

    if (
        "ERROR" in line or
        "IMU" in line or
        "TEST_PWM" in line or
        "Selected" in line or
        "Km trial" in line or
        "MENU" in line
    ):
        print(line)


def process_serial_queue(max_lines=2000):
    processed = 0

    while serial_line_queue and processed < max_lines:
        parse_serial_line(serial_line_queue.popleft())
        processed += 1


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

        "target_pwm": 0,
        "pwm_norm": 0.0,
        "pwm": 0,
        "u_cmd": 0.0,

        "mode": "cleared",
        "trial_id": 0,
        "motor_1_counts": 0,
        "motor_2_counts": 0,

        "lines_read": 0,
        "data_lines_read": 0,
        "quaternion_lines_read": 0,
    })


# =====================================================
# Main visualizer
# =====================================================

def main():
    global serial_connection

    ensure_output_files()

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
        hspace=0.75
    )

    arm_axis = figure.add_subplot(grid[:, 0], projection="3d")
    angle_axis = figure.add_subplot(grid[0, 1])
    counts_axis = figure.add_subplot(grid[1, 1])
    cable_axis = figure.add_subplot(grid[2, 1])
    speed_pwm_axis = figure.add_subplot(grid[3, 1])

    figure.subplots_adjust(
        left=0.04,
        right=0.97,
        top=0.96,
        bottom=0.09
    )

    time_history = deque(maxlen=HISTORY_POINTS)
    angle_history = deque(maxlen=HISTORY_POINTS)
    counts_history = deque(maxlen=HISTORY_POINTS)
    cable_history = deque(maxlen=HISTORY_POINTS)

    summary_pwm_history = deque(maxlen=200)
    summary_speed_history = deque(maxlen=200)

    start_time = time.monotonic()
    first_count_for_plot = None

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

    shoulder_dot, = arm_axis.plot([shoulder[0]], [shoulder[1]], [shoulder[2]], "o", markersize=7)
    upper_imu_dot, = arm_axis.plot([upper_imu[0]], [upper_imu[1]], [upper_imu[2]], "^", markersize=6)
    elbow_dot, = arm_axis.plot([elbow[0]], [elbow[1]], [elbow[2]], "o", markersize=7)
    forearm_imu_dot, = arm_axis.plot([forearm_imu[0]], [forearm_imu[1]], [forearm_imu[2]], "^", markersize=6)
    hand_dot, = arm_axis.plot([hand[0]], [hand[1]], [hand[2]], "o", markersize=7)

    upper_arm_line, = arm_axis.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], [shoulder[2], elbow[2]], linewidth=3)
    forearm_line, = arm_axis.plot([elbow[0], hand[0]], [elbow[1], hand[1]], [elbow[2], hand[2]], linewidth=3)

    angle_line, = angle_axis.plot([], [], linewidth=2, label="θ(t)")
    angle_axis.set_title("Joint Angle vs Time", fontsize=10)
    angle_axis.set_xlabel("Time (s)", fontsize=8)
    angle_axis.set_ylabel("Angle (deg)", fontsize=8)
    angle_axis.grid(True, alpha=0.3)
    angle_axis.legend(loc="upper right", fontsize=7)

    counts_line, = counts_axis.plot([], [], linewidth=2, label="M2 counts")
    counts_axis.set_title("Motor 2 Encoder Counts vs Time", fontsize=10)
    counts_axis.set_xlabel("Time (s)", fontsize=8)
    counts_axis.set_ylabel("M2 encoder counts", fontsize=8)
    counts_axis.grid(True, alpha=0.3)
    counts_axis.legend(loc="upper right", fontsize=7)

    cable_line, = cable_axis.plot([], [], linewidth=2, label="Δl")
    cable_axis.set_title("Cable Displacement From Encoder", fontsize=10)
    cable_axis.set_xlabel("Time (s)", fontsize=8)
    cable_axis.set_ylabel("Cable displacement (m)", fontsize=8)
    cable_axis.grid(True, alpha=0.3)
    cable_axis.legend(loc="upper right", fontsize=7)

    speed_pwm_line, = speed_pwm_axis.plot([], [], linewidth=0, marker="o", markersize=5, label="avg cable speed")
    speed_pwm_axis.set_title("Cable Speed vs PWM", fontsize=10)
    speed_pwm_axis.set_xlabel("PWM command", fontsize=8)
    speed_pwm_axis.set_ylabel("Cable speed (m/s)", fontsize=8)
    speed_pwm_axis.grid(True, alpha=0.3)
    speed_pwm_axis.legend(loc="upper right", fontsize=7)

    pwm_display_text = figure.text(
        0.64,
        0.975,
        "Target PWM: -- | Actual PWM: --",
        transform=figure.transFigure,
        verticalalignment="top",
        fontsize=12,
        fontweight="bold"
    )

    status_text = figure.text(
        0.02,
        0.005,
        "",
        transform=figure.transFigure,
        verticalalignment="bottom",
        fontsize=7
    )

    def reset_plot_history():
        nonlocal start_time, first_count_for_plot

        time_history.clear()
        angle_history.clear()
        counts_history.clear()
        cable_history.clear()
        first_count_for_plot = None

        start_time = time.monotonic()

        angle_line.set_data([], [])
        counts_line.set_data([], [])
        cable_line.set_data([], [])

        for ax in [angle_axis, counts_axis, cable_axis]:
            ax.set_xlim(0.0, 1.0)

        angle_axis.set_ylim(-5, 100)
        counts_axis.set_ylim(-10, 10)
        cable_axis.set_ylim(-0.01, 0.01)

    def handle_key_press(event):
        if event.key is None:
            return

        key = event.key.lower()

        print(f"Key pressed: {key}")

        if key == "n":
            reset_plot_history()
            send_serial_command("n")

        elif key == "v":
            send_serial_command("v")

        elif key in ["+", "="]:
            send_serial_command("+")

        elif key in ["-", "_"]:
            send_serial_command("-")

        elif key in ["a", "left"]:
            send_serial_command("a")

        elif key in ["d", "right"]:
            send_serial_command("d")

        elif key in ["p", "space", "s", "e", "escape"]:
            send_serial_command("e")

        elif key == "r":
            send_serial_command("r")
            reset_plot_history()

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
            stop_requested.set()
            plt.close(figure)

    figure.canvas.mpl_connect("key_press_event", handle_key_press)

    def update_summary_plot_from_file():
        if summary_file is None:
            return

        summary_path = OUTPUT_DIR / "km_summary.csv"

        if not summary_path.exists():
            return

        try:
            with open(summary_path, "r", newline="") as file:
                rows = list(csv.DictReader(file))
        except Exception:
            return

        summary_pwm_history.clear()
        summary_speed_history.clear()

        for row in rows[-200:]:
            try:
                if int(row.get("early_stop", "0")) != 0:
                    continue
                summary_pwm_history.append(float(row["pwm"]))
                summary_speed_history.append(float(row["cable_speed_mps"]))
            except Exception:
                continue

    last_summary_reload = [0.0]

    def update_plot(_frame):
        nonlocal first_count_for_plot

        process_serial_queue()

        shoulder, upper_imu, elbow, forearm_imu, hand = calculate_arm_positions()

        points = [shoulder, upper_imu, elbow, forearm_imu, hand]
        dots = [shoulder_dot, upper_imu_dot, elbow_dot, forearm_imu_dot, hand_dot]

        for dot, point in zip(dots, points):
            dot.set_data([point[0]], [point[1]])
            dot.set_3d_properties([point[2]])

        upper_arm_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
        upper_arm_line.set_3d_properties([shoulder[2], elbow[2]])

        forearm_line.set_data([elbow[0], hand[0]], [elbow[1], hand[1]])
        forearm_line.set_3d_properties([elbow[2], hand[2]])

        joint_angle = telemetry["current_angle"]

        if telemetry["quaternion_lines_read"] > 0 and abs(joint_angle) < 1e-9:
            joint_angle = quaternion_angle_degrees(telemetry["joint_quaternion"])

        current_time = time.monotonic() - start_time
        current_count = telemetry["motor_2_counts"]

        if first_count_for_plot is None:
            first_count_for_plot = current_count

        delta_counts = current_count - first_count_for_plot
        cable_disp = R_SPOOL_M * (2.0 * np.pi / COUNTS_PER_REV) * delta_counts

        time_history.append(current_time)
        angle_history.append(joint_angle)
        counts_history.append(current_count)
        cable_history.append(cable_disp)

        angle_line.set_data(list(time_history), list(angle_history))
        counts_line.set_data(list(time_history), list(counts_history))
        cable_line.set_data(list(time_history), list(cable_history))

        if len(time_history) > 1:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 1.0)

            for ax in [angle_axis, counts_axis, cable_axis]:
                ax.set_xlim(left_time, right_time)

            angle_axis.set_ylim(min(angle_history) - 5.0, max(angle_history) + 5.0)

            count_min = min(counts_history)
            count_max = max(counts_history)
            if abs(count_max - count_min) < 10:
                count_min -= 10
                count_max += 10
            counts_axis.set_ylim(count_min, count_max)

            cable_min = min(cable_history)
            cable_max = max(cable_history)
            if abs(cable_max - cable_min) < 0.001:
                cable_min -= 0.001
                cable_max += 0.001
            cable_axis.set_ylim(cable_min, cable_max)

        if time.monotonic() - last_summary_reload[0] > 1.0:
            update_summary_plot_from_file()
            last_summary_reload[0] = time.monotonic()

        speed_pwm_line.set_data(list(summary_pwm_history), list(summary_speed_history))

        if summary_pwm_history and summary_speed_history:
            pwm_min = min(summary_pwm_history) - 10
            pwm_max = max(summary_pwm_history) + 10
            speed_min = min(summary_speed_history)
            speed_max = max(summary_speed_history)

            if abs(speed_max - speed_min) < 0.001:
                speed_min -= 0.001
                speed_max += 0.001
            else:
                margin = 0.1 * abs(speed_max - speed_min)
                speed_min -= margin
                speed_max += margin

            speed_pwm_axis.set_xlim(pwm_min, pwm_max)
            speed_pwm_axis.set_ylim(speed_min, speed_max)

        pwm_display_text.set_text(
            f"Target PWM: {telemetry['target_pwm']} | Actual PWM: {telemetry['pwm']} | Mode: {telemetry['mode']}"
        )

        status_lines = [
            (
                "Keys: n start Km trial | v next PWM | +/- tune PWM | "
                "a/left manual down | d/right manual up | s/p/e/space stop | "
                "r reset | m menu | c clear | g reset graphs | q quit"
            ),
            (
                f"Trial: {telemetry['trial_id']} | Mode: {telemetry['mode']} | "
                f"θ(t): {joint_angle:.2f} deg | "
                f"Target PWM: {telemetry['target_pwm']} | "
                f"Actual PWM: {telemetry['pwm']} | u(t): {telemetry['u_cmd']:.2f} | "
                f"M2 counts: {telemetry['motor_2_counts']} | samples saved: {samples_saved}"
            ),
            f"Latest event: {latest_event}",
            f"Latest result: {latest_result}",
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
            counts_line,
            cable_line,
            speed_pwm_line,
            pwm_display_text,
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
        stop_requested.set()
        close_trial_file()

        if summary_file is not None:
            summary_file.flush()
            summary_file.close()

        if events_file is not None:
            events_file.flush()
            events_file.close()


if __name__ == "__main__":
    main()