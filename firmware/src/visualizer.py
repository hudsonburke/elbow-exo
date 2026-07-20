#!/usr/bin/env python3
"""Old visualizer layout adapted for the final dual-motor controller.

The original visualizer layout is kept:
    - A 3D arm view on the left
    - Joint angles versus time
    - Normalized control effort versus time
    - Signed PWM command versus time
    - Joint angle versus signed PWM command

Motor colors:
    - Motor 1 is blue
    - Motor 2 is orange

Keyboard controls inside the plot window:
    j       Select Motor 1
    k       Select Motor 2
    0-9     Set a fixed-angle target for the selected motor
    x       Start or stop oscillation for the selected motor
    left    Move the selected motor in reverse/down at manual PWM (sends a)
    right   Move the selected motor forward/up at manual PWM (sends d)
    s       Emergency stop both motors
    e       Emergency stop both motors
    p/space Emergency stop both motors
    r       Re-zero both IMUs and encoders
    m       Print the controller menu
    c       Clear Python-side telemetry values
    g       Reset all graph history
    q       Close the visualizer
"""

from __future__ import annotations

import re
import threading
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial


# =====================================================
# Configuration
# =====================================================

# Change this when the controller appears on a different COM port.
SERIAL_PORT = "COM8"
BAUD_RATE = 230400
SERIAL_TIMEOUT_SECONDS = 0.1

# Original visualizer window and panel sizes.
WINDOW_SIZE = (15, 11.5)
ARM_PANEL_WIDTH = 0.95
GRAPH_PANEL_WIDTH = 1.0

HISTORY_POINTS = 1000
ANIMATION_INTERVAL_MS = 35

# Approximate segment lengths used only by the 3D picture.
SHOULDER_TO_UPPER_IMU = 1.0
UPPER_IMU_TO_ELBOW = 5.25
ELBOW_TO_FOREARM_IMU = 3.0
FOREARM_IMU_TO_HAND = 1.0

IMU_SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])
UPPER_ARM_DIRECTION_SIGN = -1.0
FOREARM_DIRECTION_SIGN = -1.0

PRINT_RAW_SERIAL = False

# Motor 2 keeps a separate color so it is easy to distinguish.
MOTOR_1_COLOR = "tab:blue"
MOTOR_2_COLOR = "tab:orange"


# =====================================================
# Quaternion math used by the 3D arm picture
# =====================================================

def multiply_quaternions(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    magnitude = np.linalg.norm(q)

    if magnitude < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])

    return q / magnitude


def conjugate_quaternion(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternion(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def rotate_vector_by_quaternion(
    vector: np.ndarray,
    quaternion: np.ndarray,
) -> np.ndarray:
    quaternion = normalize_quaternion(quaternion)
    vector_quaternion = np.array(
        [0.0, vector[0], vector[1], vector[2]],
        dtype=float,
    )

    rotated = multiply_quaternions(
        multiply_quaternions(quaternion, vector_quaternion),
        conjugate_quaternion(quaternion),
    )

    return rotated[1:]


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    magnitude = np.linalg.norm(vector)

    if magnitude < 1e-12:
        return np.array([0.0, 0.0, -1.0])

    return vector / magnitude


# Rotates the IMU coordinate system into the display coordinate system.
DISPLAY_OFFSET_BASE = np.array(
    [-0.7071067811865476, 0.0, 0.7071067811865476, 0.0]
)
DISPLAY_OFFSET_EXTRA = np.array(
    [-0.7071067811865476, 0.0, 0.0, 0.7071067811865476]
)
DISPLAY_ROTATION = multiply_quaternions(
    DISPLAY_OFFSET_EXTRA,
    DISPLAY_OFFSET_BASE,
)


# =====================================================
# Serial data and telemetry
# =====================================================

FLOAT_PATTERN = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

QUATERNION_PATTERN = re.compile(
    rf"^\s*(qUpperZeroed|qForearmZeroed|qJointZeroed)\s*:\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN}),\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN})\s*$"
)

serial_line_queue: deque[str] = deque()
stop_requested = threading.Event()
serial_connection: serial.Serial | None = None


def make_default_telemetry() -> dict[str, object]:
    return {
        "upper_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "forearm_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "joint_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "board_time_s": 0.0,
        "selected_motor": "M2",
        "m1_mode": "Idle",
        "m1_target": 0.0,
        "m1_current": 0.0,
        "m1_error": 0.0,
        "m1_pwm": 0,
        "m1_u_cmd": 0.0,
        "m1_counts": 0,
        "m2_mode": "Idle",
        "m2_target": 0.0,
        "m2_current": 0.0,
        "m2_error": 0.0,
        "m2_pwm": 0,
        "m2_u_cmd": 0.0,
        "m2_counts": 0,
        "upper_angle": 0.0,
        "elbow_angle": 0.0,
        "rejected_spikes": 0,
        "lines_read": 0,
        "quaternion_lines_read": 0,
    }


telemetry = make_default_telemetry()


def reset_telemetry() -> None:
    """Clear stored values in Python without commanding the controller."""
    telemetry.clear()
    telemetry.update(make_default_telemetry())


def open_serial_port(port: str, baud_rate: int) -> serial.Serial | None:
    try:
        connection = serial.Serial(
            port,
            baud_rate,
            timeout=SERIAL_TIMEOUT_SECONDS,
        )
        print(f"Opened serial port {port} at {baud_rate} baud.")
        return connection

    except (serial.SerialException, OSError) as error:
        print(f"Could not open serial port {port}: {error}")
        return None


def send_serial_command(text: str) -> None:
    if serial_connection is None or not serial_connection.is_open:
        print("Serial port is not open.")
        return

    try:
        serial_connection.write(text.encode("utf-8"))
        print(f"Sent: {text!r}")

    except (serial.SerialException, OSError) as error:
        print(f"Could not send command: {error}")


def read_serial_lines(connection: serial.Serial) -> None:
    try:
        while not stop_requested.is_set():
            try:
                raw_line = connection.readline()
            except (serial.SerialException, OSError):
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
        except (serial.SerialException, OSError):
            pass


def parse_state_line(line: str) -> bool:
    """Read the compact STATE line sent by the dual-motor controller."""
    parts = [part.strip() for part in line.split(",")]

    if len(parts) != 20 or parts[0] != "STATE":
        return False

    try:
        telemetry["board_time_s"] = float(parts[1]) / 1000.0
        telemetry["selected_motor"] = parts[2]

        telemetry["m1_mode"] = parts[3]
        telemetry["m1_target"] = float(parts[4])
        telemetry["m1_current"] = float(parts[5])
        telemetry["m1_error"] = float(parts[6])
        telemetry["m1_pwm"] = int(parts[7])
        telemetry["m1_u_cmd"] = float(parts[8])
        telemetry["m1_counts"] = int(parts[9])

        telemetry["m2_mode"] = parts[10]
        telemetry["m2_target"] = float(parts[11])
        telemetry["m2_current"] = float(parts[12])
        telemetry["m2_error"] = float(parts[13])
        telemetry["m2_pwm"] = int(parts[14])
        telemetry["m2_u_cmd"] = float(parts[15])
        telemetry["m2_counts"] = int(parts[16])

        telemetry["upper_angle"] = float(parts[17])
        telemetry["elbow_angle"] = float(parts[18])
        telemetry["rejected_spikes"] = int(parts[19])

    except ValueError:
        return False

    return True


def parse_serial_line(line: str) -> None:
    telemetry["lines_read"] = int(telemetry["lines_read"]) + 1

    if line.startswith("STATE,"):
        parse_state_line(line)
        return

    quaternion_match = QUATERNION_PATTERN.match(line)

    if quaternion_match:
        label = quaternion_match.group(1)
        quaternion = normalize_quaternion(
            np.array(
                [
                    float(quaternion_match.group(2)),
                    float(quaternion_match.group(3)),
                    float(quaternion_match.group(4)),
                    float(quaternion_match.group(5)),
                ]
            )
        )

        if label == "qUpperZeroed":
            telemetry["upper_quaternion"] = quaternion
        elif label == "qForearmZeroed":
            telemetry["forearm_quaternion"] = quaternion
        else:
            telemetry["joint_quaternion"] = quaternion

        telemetry["quaternion_lines_read"] = (
            int(telemetry["quaternion_lines_read"]) + 1
        )
        return

    # Show controller messages such as selections, stops, and errors.
    print(line)


def process_serial_queue() -> None:
    while serial_line_queue:
        parse_serial_line(serial_line_queue.popleft())


# =====================================================
# 3D arm position calculation
# =====================================================

def calculate_arm_positions() -> tuple[np.ndarray, ...]:
    shoulder = np.array([0.0, 0.0, 0.0])
    base_axis = normalize_vector(IMU_SEGMENT_AXIS_LOCAL)

    upper_display_quaternion = normalize_quaternion(
        multiply_quaternions(
            DISPLAY_ROTATION,
            np.asarray(telemetry["upper_quaternion"]),
        )
    )

    forearm_display_quaternion = normalize_quaternion(
        multiply_quaternions(
            DISPLAY_ROTATION,
            np.asarray(telemetry["forearm_quaternion"]),
        )
    )

    upper_direction = normalize_vector(
        rotate_vector_by_quaternion(
            base_axis * UPPER_ARM_DIRECTION_SIGN,
            upper_display_quaternion,
        )
    )

    forearm_direction = normalize_vector(
        rotate_vector_by_quaternion(
            base_axis * FOREARM_DIRECTION_SIGN,
            forearm_display_quaternion,
        )
    )

    upper_imu = shoulder + SHOULDER_TO_UPPER_IMU * upper_direction

    elbow = shoulder + (
        SHOULDER_TO_UPPER_IMU + UPPER_IMU_TO_ELBOW
    ) * upper_direction

    forearm_imu = elbow + ELBOW_TO_FOREARM_IMU * forearm_direction

    hand = elbow + (
        ELBOW_TO_FOREARM_IMU + FOREARM_IMU_TO_HAND
    ) * forearm_direction

    return shoulder, upper_imu, elbow, forearm_imu, hand


# =====================================================
# Main visualizer
# =====================================================

def main() -> None:
    global serial_connection

    serial_connection = open_serial_port(SERIAL_PORT, BAUD_RATE)

    if serial_connection is None:
        return

    serial_thread = threading.Thread(
        target=read_serial_lines,
        args=(serial_connection,),
        daemon=True,
    )
    serial_thread.start()

    # Keep the original window layout: 3D arm on the left and four graphs.
    figure = plt.figure(figsize=WINDOW_SIZE)

    grid = figure.add_gridspec(
        4,
        2,
        width_ratios=[ARM_PANEL_WIDTH, GRAPH_PANEL_WIDTH],
        height_ratios=[1.0, 1.0, 1.0, 1.0],
        wspace=0.35,
        hspace=0.80,
    )

    arm_axis = figure.add_subplot(grid[:, 0], projection="3d")
    angle_axis = figure.add_subplot(grid[0, 1])
    pwm_norm_axis = figure.add_subplot(grid[1, 1])
    u_time_axis = figure.add_subplot(grid[2, 1])
    angle_u_axis = figure.add_subplot(grid[3, 1])

    figure.subplots_adjust(
        left=0.04,
        right=0.97,
        top=0.96,
        bottom=0.10,
    )

    # Separate history is kept for both motors.
    time_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    m1_angle_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m1_target_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_angle_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_target_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    m1_pwm_norm_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_pwm_norm_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    m1_u_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_u_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    m1_angle_for_u_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_angle_for_u_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    start_time = time.monotonic()

    # ---------------------
    # Original 3D arm panel
    # ---------------------

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

    shoulder, upper_imu, elbow, forearm_imu, hand = (
        calculate_arm_positions()
    )

    shoulder_dot, = arm_axis.plot(
        [shoulder[0]],
        [shoulder[1]],
        [shoulder[2]],
        "o",
        markersize=7,
        label="Shoulder",
    )

    upper_imu_dot, = arm_axis.plot(
        [upper_imu[0]],
        [upper_imu[1]],
        [upper_imu[2]],
        "^",
        markersize=6,
        label="Upper IMU / Motor 1",
    )

    elbow_dot, = arm_axis.plot(
        [elbow[0]],
        [elbow[1]],
        [elbow[2]],
        "o",
        markersize=7,
        label="Elbow",
    )

    forearm_imu_dot, = arm_axis.plot(
        [forearm_imu[0]],
        [forearm_imu[1]],
        [forearm_imu[2]],
        "^",
        markersize=6,
        label="Forearm IMU / Motor 2",
    )

    hand_dot, = arm_axis.plot(
        [hand[0]],
        [hand[1]],
        [hand[2]],
        "o",
        markersize=7,
        label="Hand",
    )

    upper_arm_line, = arm_axis.plot(
        [shoulder[0], elbow[0]],
        [shoulder[1], elbow[1]],
        [shoulder[2], elbow[2]],
        linewidth=3,
        label="Motor 1 segment",
    )

    forearm_line, = arm_axis.plot(
        [elbow[0], hand[0]],
        [elbow[1], hand[1]],
        [elbow[2], hand[2]],
        linewidth=3,
        label="Motor 2 segment",
    )

    # ---------------------
    # Graph 1: angles versus time
    # ---------------------

    m1_angle_line, = angle_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_1_COLOR,
        label="M1 current",
    )
    m1_target_line, = angle_axis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=MOTOR_1_COLOR,
        label="M1 target",
    )
    m2_angle_line, = angle_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_2_COLOR,
        label="M2 current",
    )
    m2_target_line, = angle_axis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=MOTOR_2_COLOR,
        label="M2 target",
    )

    angle_axis.set_title("Output θ(t): Motor Angles vs Time", fontsize=10)
    angle_axis.set_xlabel("Time t (s)", fontsize=8)
    angle_axis.set_ylabel("Angle θ(t) (degrees)", fontsize=8)
    angle_axis.grid(True, alpha=0.3)
    angle_axis.set_xlim(0.0, 1.0)
    angle_axis.set_ylim(-5, 95)
    angle_axis.legend(loc="upper right", fontsize=7, ncol=2)

    # ---------------------
    # Graph 2: normalized PWM versus time
    # ---------------------

    m1_pwm_norm_line, = pwm_norm_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_1_COLOR,
        label="M1 |PWM| / 255",
    )
    m2_pwm_norm_line, = pwm_norm_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_2_COLOR,
        label="M2 |PWM| / 255",
    )

    pwm_norm_axis.set_title(
        "Normalized Control Effort |u(t)|",
        fontsize=10,
    )
    pwm_norm_axis.set_xlabel("Time t (s)", fontsize=8)
    pwm_norm_axis.set_ylabel("Normalized PWM", fontsize=8)
    pwm_norm_axis.grid(True, alpha=0.3)
    pwm_norm_axis.set_xlim(0.0, 1.0)
    pwm_norm_axis.set_ylim(-0.05, 1.05)
    pwm_norm_axis.legend(loc="upper right", fontsize=7)

    # ---------------------
    # Graph 3: signed PWM versus time
    # ---------------------

    m1_u_time_line, = u_time_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_1_COLOR,
        label="M1 u(t)",
    )
    m2_u_time_line, = u_time_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_2_COLOR,
        label="M2 u(t)",
    )

    u_time_axis.set_title(
        "Input u(t): Signed PWM Command vs Time",
        fontsize=10,
    )
    u_time_axis.set_xlabel("Time t (s)", fontsize=8)
    u_time_axis.set_ylabel("Signed PWM command", fontsize=8)
    u_time_axis.grid(True, alpha=0.3)
    u_time_axis.set_xlim(0.0, 1.0)
    u_time_axis.set_ylim(-260, 260)
    u_time_axis.legend(loc="upper right", fontsize=7)

    # ---------------------
    # Graph 4: angle versus signed PWM
    # ---------------------

    m1_angle_u_line, = angle_u_axis.plot(
        [],
        [],
        linewidth=1.5,
        marker=".",
        markersize=3,
        color=MOTOR_1_COLOR,
        label="M1 θ(t) vs u(t)",
    )
    m2_angle_u_line, = angle_u_axis.plot(
        [],
        [],
        linewidth=1.5,
        marker=".",
        markersize=3,
        color=MOTOR_2_COLOR,
        label="M2 θ(t) vs u(t)",
    )

    angle_u_axis.set_title(
        "Input-Output Plot: θ(t) vs u(t)",
        fontsize=10,
    )
    angle_u_axis.set_xlabel("Input u(t): signed PWM command", fontsize=8)
    angle_u_axis.set_ylabel("Output θ(t): angle (degrees)", fontsize=8)
    angle_u_axis.grid(True, alpha=0.3)
    angle_u_axis.set_xlim(-260, 260)
    angle_u_axis.set_ylim(-5, 95)
    angle_u_axis.legend(loc="upper right", fontsize=7)

    # Original small status area at the bottom of the window.
    status_text = figure.text(
        0.02,
        0.005,
        "",
        transform=figure.transFigure,
        verticalalignment="bottom",
        fontsize=7,
    )

    def reset_plot_history() -> None:
        nonlocal start_time

        time_history.clear()

        m1_angle_history.clear()
        m1_target_history.clear()
        m2_angle_history.clear()
        m2_target_history.clear()

        m1_pwm_norm_history.clear()
        m2_pwm_norm_history.clear()

        m1_u_history.clear()
        m2_u_history.clear()

        m1_angle_for_u_history.clear()
        m2_angle_for_u_history.clear()

        start_time = time.monotonic()

        for line in (
            m1_angle_line,
            m1_target_line,
            m2_angle_line,
            m2_target_line,
            m1_pwm_norm_line,
            m2_pwm_norm_line,
            m1_u_time_line,
            m2_u_time_line,
            m1_angle_u_line,
            m2_angle_u_line,
        ):
            line.set_data([], [])

        angle_axis.set_xlim(0.0, 1.0)
        pwm_norm_axis.set_xlim(0.0, 1.0)
        u_time_axis.set_xlim(0.0, 1.0)

    def handle_key_press(event) -> None:
        if event.key is None:
            return

        key = event.key.lower()
        print(f"Key pressed: {key}")

        if key in [str(number) for number in range(10)]:
            send_serial_command(key)

        elif key == "j":
            send_serial_command("j")

        elif key == "k":
            send_serial_command("k")

        elif key == "x":
            send_serial_command("x")

        elif key == "left":
            # Send one simple command byte instead of an ANSI escape sequence.
            # This is more reliable when commands pass through pyserial.
            send_serial_command("a")
            print("Manual reverse/down command sent to selected motor.")

        elif key == "right":
            # The controller interprets d as manual forward/up movement.
            send_serial_command("d")
            print("Manual forward/up command sent to selected motor.")

        elif key in ["s", "space", "p", "e"]:
            send_serial_command("s")

        elif key == "r":
            send_serial_command("r")

        elif key == "m":
            send_serial_command("m")

        elif key == "c":
            reset_telemetry()
            print("Python-side telemetry cleared.")

        elif key == "g":
            reset_plot_history()
            print("Graph history reset.")

        elif key == "q":
            stop_requested.set()
            plt.close(figure)

    figure.canvas.mpl_connect("key_press_event", handle_key_press)

    def update_plot(_frame):
        process_serial_queue()

        # Update the 3D arm using the newest IMU quaternions.
        shoulder, upper_imu, elbow, forearm_imu, hand = (
            calculate_arm_positions()
        )

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
            [shoulder[1], elbow[1]],
        )
        upper_arm_line.set_3d_properties(
            [shoulder[2], elbow[2]]
        )

        forearm_line.set_data(
            [elbow[0], hand[0]],
            [elbow[1], hand[1]],
        )
        forearm_line.set_3d_properties(
            [elbow[2], hand[2]]
        )

        elapsed_time = time.monotonic() - start_time

        m1_current = float(telemetry["m1_current"])
        m1_target = float(telemetry["m1_target"])
        m1_u = float(telemetry["m1_u_cmd"])
        m1_pwm_norm = min(abs(float(telemetry["m1_pwm"])) / 255.0, 1.0)

        m2_current = float(telemetry["m2_current"])
        m2_target = float(telemetry["m2_target"])
        m2_u = float(telemetry["m2_u_cmd"])
        m2_pwm_norm = min(abs(float(telemetry["m2_pwm"])) / 255.0, 1.0)

        time_history.append(elapsed_time)

        m1_angle_history.append(m1_current)
        m1_target_history.append(m1_target)
        m2_angle_history.append(m2_current)
        m2_target_history.append(m2_target)

        m1_pwm_norm_history.append(m1_pwm_norm)
        m2_pwm_norm_history.append(m2_pwm_norm)

        m1_u_history.append(m1_u)
        m2_u_history.append(m2_u)

        m1_angle_for_u_history.append(m1_current)
        m2_angle_for_u_history.append(m2_current)

        # Graph 1: current and target angles.
        m1_angle_line.set_data(time_history, m1_angle_history)
        m1_target_line.set_data(time_history, m1_target_history)
        m2_angle_line.set_data(time_history, m2_angle_history)
        m2_target_line.set_data(time_history, m2_target_history)

        # Graph 2: normalized control effort.
        m1_pwm_norm_line.set_data(time_history, m1_pwm_norm_history)
        m2_pwm_norm_line.set_data(time_history, m2_pwm_norm_history)

        # Graph 3: signed PWM command.
        m1_u_time_line.set_data(time_history, m1_u_history)
        m2_u_time_line.set_data(time_history, m2_u_history)

        # Graph 4: input-output relationship.
        m1_angle_u_line.set_data(m1_u_history, m1_angle_for_u_history)
        m2_angle_u_line.set_data(m2_u_history, m2_angle_for_u_history)

        if time_history:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 0.5)

            angle_axis.set_xlim(left_time, right_time)
            pwm_norm_axis.set_xlim(left_time, right_time)
            u_time_axis.set_xlim(left_time, right_time)

        status_lines = [
            (
                "Keys: j select M1 | k select M2 | left/right manual | "
                "0-9 target | x oscillation | s/space stop | r zero | "
                "g reset | q quit"
            ),
            (
                f"Selected: {telemetry['selected_motor']} | "
                f"M1 {telemetry['m1_mode']}: target "
                f"{float(telemetry['m1_target']):.1f}°, current "
                f"{float(telemetry['m1_current']):.1f}°, error "
                f"{float(telemetry['m1_error']):.1f}°, PWM "
                f"{float(telemetry['m1_u_cmd']):.0f}, counts "
                f"{telemetry['m1_counts']}"
            ),
            (
                f"M2 {telemetry['m2_mode']}: target "
                f"{float(telemetry['m2_target']):.1f}°, current "
                f"{float(telemetry['m2_current']):.1f}°, error "
                f"{float(telemetry['m2_error']):.1f}°, PWM "
                f"{float(telemetry['m2_u_cmd']):.0f}, counts "
                f"{telemetry['m2_counts']} | Rejected spikes: "
                f"{telemetry['rejected_spikes']}"
            ),
        ]

        status_text.set_text("\n".join(status_lines))

        return (
            *dots,
            upper_arm_line,
            forearm_line,
            m1_angle_line,
            m1_target_line,
            m2_angle_line,
            m2_target_line,
            m1_pwm_norm_line,
            m2_pwm_norm_line,
            m1_u_time_line,
            m2_u_time_line,
            m1_angle_u_line,
            m2_angle_u_line,
            status_text,
        )

    visualizer_animation = animation.FuncAnimation(
        figure,
        update_plot,
        interval=ANIMATION_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    # Keep a reference so Python does not delete the animation object.
    figure._visualizer_animation = visualizer_animation

    try:
        plt.show()
    finally:
        stop_requested.set()


if __name__ == "__main__":
    main()