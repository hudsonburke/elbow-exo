#!/usr/bin/env python3

# Keyboard controls inside the plot window:
#   0-9        send selected target angle
#   x          start/stop sinusoidal trajectory in C++ code
#   left       manual motor reverse
#   right      manual motor forward
#   p/space    pause both motors, same as sending 's' in C++ code
#   r          recalibrate/zero joint angle
#   m          print C++ menu
#   c          clear Python-side stored telemetry
#   g          reset the graphs
#   n          start a new CSV trial
#   q          quit visualizer

import re
import csv
import threading
from datetime import datetime
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial


# =====================================================
# Configuration
# =====================================================

SERIAL_PORT = "COM8"
BAUD_RATE = 115200

SAVE_CSV = True
CSV_BASE_NAME = "elbow_angle_encoder_trial"

WINDOW_SIZE = (15, 11.2)

ARM_PANEL_WIDTH = 0.95
GRAPH_PANEL_WIDTH = 1.0

HISTORY_POINTS = 500
ANIMATION_INTERVAL_MS = 35

SHOULDER_TO_UPPER_IMU = 1.0
UPPER_IMU_TO_ELBOW = 5.25
ELBOW_TO_FOREARM_IMU = 3.0
FOREARM_IMU_TO_HAND = 1.0

IMU_SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

UPPER_ARM_DIRECTION_SIGN = -1.0
FOREARM_DIRECTION_SIGN = -1.0

PRINT_RAW_SERIAL = False


# =====================================================
# CSV helper
# =====================================================

def make_csv_filename(trial_number):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{CSV_BASE_NAME}_{trial_number:02d}_{timestamp}.csv"


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

DATA_LINE_PATTERN = re.compile(
    rf"^DATA,\s*(\d+),\s*({FLOAT_PATTERN}),\s*(-?\d+),\s*(-?\d+)"
)

QUATERNION_LINE_PATTERN = re.compile(
    rf"^\s*(qUpperZeroed|qForearmZeroed|qJointZeroed)\s*:\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN}),\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN})"
)


# =====================================================
# Runtime state
# =====================================================

serial_line_queue = deque()
data_sample_queue = deque()

stop_requested = threading.Event()
serial_connection = None

telemetry = {
    "upper_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
    "forearm_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
    "joint_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),

    "arduino_time_ms": 0,
    "current_angle": 0.0,
    "motor_1_counts": 0,
    "motor_2_counts": 0,

    "lines_read": 0,
    "quaternion_lines_read": 0,
    "data_lines_read": 0,
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
# Serial parsing
# =====================================================

def parse_serial_line(line):
    telemetry["lines_read"] += 1

    data_match = DATA_LINE_PATTERN.search(line)

    if data_match:
        sample = {
            "arduino_time_ms": int(data_match.group(1)),
            "theta_deg": float(data_match.group(2)),
            "m1_counts": int(data_match.group(3)),
            "m2_counts": int(data_match.group(4)),
        }

        telemetry["arduino_time_ms"] = sample["arduino_time_ms"]
        telemetry["current_angle"] = sample["theta_deg"]
        telemetry["motor_1_counts"] = sample["m1_counts"]
        telemetry["motor_2_counts"] = sample["m2_counts"]
        telemetry["data_lines_read"] += 1

        data_sample_queue.append(sample)
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

        "arduino_time_ms": 0,
        "current_angle": 0.0,
        "motor_1_counts": 0,
        "motor_2_counts": 0,

        "lines_read": 0,
        "quaternion_lines_read": 0,
        "data_lines_read": 0,
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
    m1_axis = figure.add_subplot(grid[1, 1])
    m2_axis = figure.add_subplot(grid[2, 1])
    angle_m2_axis = figure.add_subplot(grid[3, 1])

    figure.subplots_adjust(
        left=0.04,
        right=0.97,
        top=0.96,
        bottom=0.09
    )

    time_history = deque(maxlen=HISTORY_POINTS)
    angle_history = deque(maxlen=HISTORY_POINTS)
    m1_count_history = deque(maxlen=HISTORY_POINTS)
    m2_count_history = deque(maxlen=HISTORY_POINTS)

    graph_start_arduino_ms = None

    # =====================================================
    # CSV setup
    # =====================================================

    csv_file = None
    csv_writer = None
    trial_number = 0
    current_csv_filename = "CSV_OFF"
    trial_start_arduino_ms = None
    rows_since_flush = 0
    csv_rows_this_trial = 0

    def start_new_csv_trial():
        nonlocal csv_file
        nonlocal csv_writer
        nonlocal trial_number
        nonlocal current_csv_filename
        nonlocal trial_start_arduino_ms
        nonlocal rows_since_flush
        nonlocal csv_rows_this_trial

        if not SAVE_CSV:
            return

        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
            print(f"Closed CSV file: {current_csv_filename}")

        serial_line_queue.clear()
        data_sample_queue.clear()

        try:
            if serial_connection is not None and serial_connection.is_open:
                serial_connection.reset_input_buffer()

        except Exception:
            pass

        trial_number += 1
        current_csv_filename = make_csv_filename(trial_number)

        csv_file = open(current_csv_filename, mode="w", newline="")
        csv_writer = csv.writer(csv_file)

        csv_writer.writerow([
            "time_s",
            "theta_deg",
            "m1_counts",
            "m2_counts"
        ])

        trial_start_arduino_ms = None
        rows_since_flush = 0
        csv_rows_this_trial = 0

        print(f"Started new CSV trial: {current_csv_filename}")

    start_new_csv_trial()

    # =====================================================
    # 3D arm setup
    # =====================================================

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

    # =====================================================
    # Graph setup
    # =====================================================

    angle_line, = angle_axis.plot(
        [],
        [],
        linewidth=2,
        label="θ(t)"
    )

    angle_axis.set_title("Joint Angle θ(t) vs Time", fontsize=10)
    angle_axis.set_xlabel("Time t (s)", fontsize=7, labelpad=1)
    angle_axis.set_ylabel("θ(t) [deg]", fontsize=8)
    angle_axis.grid(True, alpha=0.3)
    angle_axis.set_ylim(-5, 95)
    angle_axis.legend(loc="upper right", fontsize=7)

    m1_line, = m1_axis.plot(
        [],
        [],
        linewidth=2,
        label="M1 counts"
    )

    m1_axis.set_title("Motor 1 Encoder Counts vs Time", fontsize=10)
    m1_axis.set_xlabel("Time t (s)", fontsize=7, labelpad=1)
    m1_axis.set_ylabel("M1 counts", fontsize=8)
    m1_axis.grid(True, alpha=0.3)
    m1_axis.legend(loc="upper right", fontsize=7)

    m2_line, = m2_axis.plot(
        [],
        [],
        linewidth=2,
        label="M2 counts"
    )

    m2_axis.set_title("Motor 2 Encoder Counts vs Time", fontsize=10)
    m2_axis.set_xlabel("Time t (s)", fontsize=7, labelpad=1)
    m2_axis.set_ylabel("M2 counts", fontsize=8)
    m2_axis.grid(True, alpha=0.3)
    m2_axis.legend(loc="upper right", fontsize=7)

    angle_m2_line, = angle_m2_axis.plot(
        [],
        [],
        linewidth=2,
        marker=".",
        markersize=3,
        label="θ vs M2 counts"
    )

    angle_m2_axis.set_title("Angle vs Motor 2 Encoder Counts", fontsize=10)
    angle_m2_axis.set_xlabel("M2 encoder counts", fontsize=7, labelpad=1)
    angle_m2_axis.set_ylabel("θ(t) [deg]", fontsize=8)
    angle_m2_axis.grid(True, alpha=0.3)
    angle_m2_axis.legend(loc="upper right", fontsize=7)

    status_text = figure.text(
        0.02,
        0.005,
        "",
        transform=figure.transFigure,
        verticalalignment="bottom",
        fontsize=7
    )

    def set_dynamic_y_limits(axis, values, min_padding=1.0):
        if not values:
            return

        min_value = min(values)
        max_value = max(values)

        spread = max_value - min_value

        if spread < 1e-9:
            spread = 1.0

        padding = max(min_padding, 0.10 * spread)

        axis.set_ylim(min_value - padding, max_value + padding)

    def set_dynamic_x_limits(axis, values, min_padding=1.0):
        if not values:
            return

        min_value = min(values)
        max_value = max(values)

        spread = max_value - min_value

        if spread < 1e-9:
            spread = 1.0

        padding = max(min_padding, 0.10 * spread)

        axis.set_xlim(min_value - padding, max_value + padding)

    def reset_plot_history():
        nonlocal graph_start_arduino_ms

        time_history.clear()
        angle_history.clear()
        m1_count_history.clear()
        m2_count_history.clear()

        serial_line_queue.clear()
        data_sample_queue.clear()

        graph_start_arduino_ms = None

        angle_line.set_data([], [])
        angle_axis.set_xlim(0.0, 1.0)
        angle_axis.set_ylim(-5, 95)

        m1_line.set_data([], [])
        m1_axis.set_xlim(0.0, 1.0)
        m1_axis.set_ylim(-10, 10)

        m2_line.set_data([], [])
        m2_axis.set_xlim(0.0, 1.0)
        m2_axis.set_ylim(-10, 10)

        angle_m2_line.set_data([], [])
        angle_m2_axis.set_xlim(-10, 10)
        angle_m2_axis.set_ylim(-5, 95)

    def handle_key_press(event):
        if event.key is None:
            return

        key = event.key.lower()

        print(f"Key pressed: {key}")

        if key in [str(i) for i in range(10)]:
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

        elif key == "n":
            reset_plot_history()
            start_new_csv_trial()
            print("New CSV trial started.")

        elif key == "q":
            stop_requested.set()
            plt.close(figure)

    figure.canvas.mpl_connect("key_press_event", handle_key_press)

    def update_plot(_frame):
        nonlocal graph_start_arduino_ms
        nonlocal trial_start_arduino_ms
        nonlocal rows_since_flush
        nonlocal csv_rows_this_trial

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

        # =====================================================
        # Process all new DATA samples
        # This saves every DATA line to CSV, not just one per graph frame.
        # =====================================================

        while data_sample_queue:
            sample = data_sample_queue.popleft()

            if graph_start_arduino_ms is None:
                graph_start_arduino_ms = sample["arduino_time_ms"]

            graph_time_s = (
                sample["arduino_time_ms"] - graph_start_arduino_ms
            ) / 1000.0

            time_history.append(graph_time_s)
            angle_history.append(sample["theta_deg"])
            m1_count_history.append(sample["m1_counts"])
            m2_count_history.append(sample["m2_counts"])

            if SAVE_CSV and csv_writer is not None:
                if trial_start_arduino_ms is None:
                    trial_start_arduino_ms = sample["arduino_time_ms"]

                csv_time_s = (
                    sample["arduino_time_ms"] - trial_start_arduino_ms
                ) / 1000.0

                csv_writer.writerow([
                    csv_time_s,
                    sample["theta_deg"],
                    sample["m1_counts"],
                    sample["m2_counts"]
                ])

                rows_since_flush += 1
                csv_rows_this_trial += 1

                if rows_since_flush >= 20:
                    csv_file.flush()
                    rows_since_flush = 0

        # =====================================================
        # Update graphs
        # =====================================================

        angle_line.set_data(list(time_history), list(angle_history))
        m1_line.set_data(list(time_history), list(m1_count_history))
        m2_line.set_data(list(time_history), list(m2_count_history))
        angle_m2_line.set_data(list(m2_count_history), list(angle_history))

        if time_history:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 0.2)

            angle_axis.set_xlim(left_time, right_time)
            m1_axis.set_xlim(left_time, right_time)
            m2_axis.set_xlim(left_time, right_time)

        if angle_history:
            set_dynamic_y_limits(angle_axis, angle_history, min_padding=2.0)
            set_dynamic_y_limits(angle_m2_axis, angle_history, min_padding=2.0)

        if m1_count_history:
            set_dynamic_y_limits(m1_axis, m1_count_history, min_padding=10.0)

        if m2_count_history:
            set_dynamic_y_limits(m2_axis, m2_count_history, min_padding=10.0)
            set_dynamic_x_limits(angle_m2_axis, m2_count_history, min_padding=10.0)

        latest_time = time_history[-1] if time_history else 0.0
        joint_angle = telemetry["current_angle"]

        if telemetry["data_lines_read"] == 0 and telemetry["quaternion_lines_read"] > 0:
            joint_angle = quaternion_angle_degrees(
                telemetry["joint_quaternion"]
            )

        status_lines = [
            (
                "Keys: 0-9 target | x trajectory | left/right manual | "
                "p/space pause | r recalibrate | m menu | "
                "c clear | g reset graphs | n new CSV trial | q quit"
            ),
            (
                f"t: {latest_time:.3f} s | "
                f"θ(t): {joint_angle:.2f} deg | "
                f"M1Counts: {telemetry['motor_1_counts']} | "
                f"M2Counts: {telemetry['motor_2_counts']} | "
                f"DATA samples read: {telemetry['data_lines_read']}"
            ),
            (
                f"CSV trial: {trial_number} | "
                f"Rows saved this trial: {csv_rows_this_trial} | "
                f"File: {current_csv_filename if SAVE_CSV else 'OFF'}"
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
            m1_line,
            m2_line,
            angle_m2_line,
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

        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
            print(f"Saved CSV file: {current_csv_filename}")


if __name__ == "__main__":
    main()