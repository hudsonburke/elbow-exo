#!/usr/bin/env python3
"""Old visualizer layout adapted for the final dual-motor controller.

The visualizer keeps the main final-product displays:
    - A smaller 3D arm view on the left
    - Joint angles versus time
    - Angular velocity versus time
    - Signed PWM command versus time

The velocity graph uses the exact desired and measured velocity values sent by
an updated controller. If those VELOCITY messages are unavailable, the program
uses a numerical derivative as a clearly labeled fallback.

Motor colors:
    - Motor 1 is blue
    - Motor 2 is orange

Keyboard controls inside the plot window:
    j       Select Motor 1
    k       Select Motor 2
    0-9     Set a fixed-angle target for the selected motor
    x       Start or stop oscillation for the selected motor
    left/a  Move the selected motor in reverse/down at manual PWM
    right/d Move the selected motor forward/up at manual PWM
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

# Reduced layout: the 3D arm is slightly smaller and the graphs get more width.
WINDOW_SIZE = (15, 10.5)
ARM_PANEL_WIDTH = 0.72
GRAPH_PANEL_WIDTH = 1.28

HISTORY_POINTS = 1200
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

# Used only when an older controller does not send VELOCITY telemetry.
MEASURED_VELOCITY_FILTER_ALPHA = 0.25
TARGET_VELOCITY_FILTER_ALPHA = 0.50
MIN_VALID_DT_SECONDS = 0.001
MAX_VALID_DT_SECONDS = 0.250
VELOCITY_TIMESTAMP_TOLERANCE_SECONDS = 0.005


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
        "m1_velocity": 0.0,
        "m1_target_velocity": 0.0,
        "m2_mode": "Idle",
        "m2_target": 0.0,
        "m2_current": 0.0,
        "m2_error": 0.0,
        "m2_pwm": 0,
        "m2_u_cmd": 0.0,
        "m2_counts": 0,
        "m2_velocity": 0.0,
        "m2_target_velocity": 0.0,
        "velocity_source": "Waiting for telemetry",
        "upper_angle": 0.0,
        "elbow_angle": 0.0,
        "rejected_spikes": 0,
        "lines_read": 0,
        "quaternion_lines_read": 0,
    }


telemetry = make_default_telemetry()

# Previous samples are kept only for the older-controller fallback.
_previous_time_s: float | None = None
_previous_m1_angle: float = 0.0
_previous_m1_target: float = 0.0
_previous_m2_angle: float = 0.0
_previous_m2_target: float = 0.0
_last_controller_velocity_time_s: float | None = None


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
        serial_connection.flush()
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


def low_pass(previous: float, new_value: float, alpha: float) -> float:
    """Apply a simple low-pass filter to a fallback velocity estimate."""
    return alpha * new_value + (1.0 - alpha) * previous


def parse_velocity_line(line: str) -> bool:
    """Read exact velocity values used inside the controller.

    Expected format:
        VELOCITY,time_ms,
        m1_desired_velocity,m1_measured_velocity,
        m2_desired_velocity,m2_measured_velocity
    """
    global _last_controller_velocity_time_s

    parts = [part.strip() for part in line.split(",")]

    if len(parts) != 6 or parts[0] != "VELOCITY":
        return False

    try:
        time_s = float(parts[1]) / 1000.0
        telemetry["m1_target_velocity"] = float(parts[2])
        telemetry["m1_velocity"] = float(parts[3])
        telemetry["m2_target_velocity"] = float(parts[4])
        telemetry["m2_velocity"] = float(parts[5])
    except ValueError:
        return False

    telemetry["velocity_source"] = "Controller internal values"
    _last_controller_velocity_time_s = time_s
    return True


def parse_state_line(line: str) -> bool:
    """Read the compact STATE line sent by the dual-motor controller.

    The updated controller sends a matching VELOCITY line immediately before
    this line. When that message is missing, velocity is estimated from the
    change in angle divided by the change in time.
    """
    global _previous_time_s
    global _previous_m1_angle
    global _previous_m1_target
    global _previous_m2_angle
    global _previous_m2_target

    parts = [part.strip() for part in line.split(",")]

    if len(parts) != 20 or parts[0] != "STATE":
        return False

    try:
        time_s = float(parts[1]) / 1000.0
        selected_motor = parts[2]

        m1_mode = parts[3]
        m1_target = float(parts[4])
        m1_current = float(parts[5])
        m1_error = float(parts[6])
        m1_pwm = int(parts[7])
        m1_u_cmd = float(parts[8])
        m1_counts = int(parts[9])

        m2_mode = parts[10]
        m2_target = float(parts[11])
        m2_current = float(parts[12])
        m2_error = float(parts[13])
        m2_pwm = int(parts[14])
        m2_u_cmd = float(parts[15])
        m2_counts = int(parts[16])

        upper_angle = float(parts[17])
        elbow_angle = float(parts[18])
        rejected_spikes = int(parts[19])
    except ValueError:
        return False

    controller_velocity_is_current = (
        _last_controller_velocity_time_s is not None
        and abs(_last_controller_velocity_time_s - time_s)
        <= VELOCITY_TIMESTAMP_TOLERANCE_SECONDS
    )

    if not controller_velocity_is_current and _previous_time_s is not None:
        dt = time_s - _previous_time_s

        if MIN_VALID_DT_SECONDS <= dt <= MAX_VALID_DT_SECONDS:
            raw_m1_velocity = (m1_current - _previous_m1_angle) / dt
            raw_m1_target_velocity = (m1_target - _previous_m1_target) / dt
            raw_m2_velocity = (m2_current - _previous_m2_angle) / dt
            raw_m2_target_velocity = (m2_target - _previous_m2_target) / dt

            telemetry["m1_velocity"] = low_pass(
                float(telemetry["m1_velocity"]),
                raw_m1_velocity,
                MEASURED_VELOCITY_FILTER_ALPHA,
            )
            telemetry["m1_target_velocity"] = low_pass(
                float(telemetry["m1_target_velocity"]),
                raw_m1_target_velocity,
                TARGET_VELOCITY_FILTER_ALPHA,
            )
            telemetry["m2_velocity"] = low_pass(
                float(telemetry["m2_velocity"]),
                raw_m2_velocity,
                MEASURED_VELOCITY_FILTER_ALPHA,
            )
            telemetry["m2_target_velocity"] = low_pass(
                float(telemetry["m2_target_velocity"]),
                raw_m2_target_velocity,
                TARGET_VELOCITY_FILTER_ALPHA,
            )
            telemetry["velocity_source"] = "Python numerical fallback"

    telemetry["board_time_s"] = time_s
    telemetry["selected_motor"] = selected_motor

    telemetry["m1_mode"] = m1_mode
    telemetry["m1_target"] = m1_target
    telemetry["m1_current"] = m1_current
    telemetry["m1_error"] = m1_error
    telemetry["m1_pwm"] = m1_pwm
    telemetry["m1_u_cmd"] = m1_u_cmd
    telemetry["m1_counts"] = m1_counts

    telemetry["m2_mode"] = m2_mode
    telemetry["m2_target"] = m2_target
    telemetry["m2_current"] = m2_current
    telemetry["m2_error"] = m2_error
    telemetry["m2_pwm"] = m2_pwm
    telemetry["m2_u_cmd"] = m2_u_cmd
    telemetry["m2_counts"] = m2_counts

    telemetry["upper_angle"] = upper_angle
    telemetry["elbow_angle"] = elbow_angle
    telemetry["rejected_spikes"] = rejected_spikes

    _previous_time_s = time_s
    _previous_m1_angle = m1_current
    _previous_m1_target = m1_target
    _previous_m2_angle = m2_current
    _previous_m2_target = m2_target

    return True


def parse_serial_line(line: str) -> None:
    telemetry["lines_read"] = int(telemetry["lines_read"]) + 1

    if line.startswith("VELOCITY,"):
        parse_velocity_line(line)
        return

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

    # Three time plots remain. The narrower left column makes the 3D arm smaller.
    figure = plt.figure(figsize=WINDOW_SIZE)

    grid = figure.add_gridspec(
        3,
        2,
        width_ratios=[ARM_PANEL_WIDTH, GRAPH_PANEL_WIDTH],
        height_ratios=[1.0, 1.0, 1.0],
        wspace=0.32,
        hspace=0.62,
    )

    arm_axis = figure.add_subplot(grid[:, 0], projection="3d")
    angle_axis = figure.add_subplot(grid[0, 1])
    velocity_axis = figure.add_subplot(grid[1, 1])
    u_time_axis = figure.add_subplot(grid[2, 1])

    figure.subplots_adjust(
        left=0.04,
        right=0.97,
        top=0.96,
        bottom=0.11,
    )

    # Separate history is kept for both motors.
    time_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    m1_angle_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m1_target_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_angle_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_target_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    m1_velocity_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m1_target_velocity_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_velocity_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_target_velocity_history: deque[float] = deque(maxlen=HISTORY_POINTS)

    m1_u_history: deque[float] = deque(maxlen=HISTORY_POINTS)
    m2_u_history: deque[float] = deque(maxlen=HISTORY_POINTS)

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
    # Graph 2: angular velocity versus time
    # ---------------------

    m1_velocity_line, = velocity_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_1_COLOR,
        label="M1 measured velocity",
    )
    m1_target_velocity_line, = velocity_axis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=MOTOR_1_COLOR,
        label="M1 desired velocity",
    )
    m2_velocity_line, = velocity_axis.plot(
        [],
        [],
        linewidth=2,
        color=MOTOR_2_COLOR,
        label="M2 measured velocity",
    )
    m2_target_velocity_line, = velocity_axis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=MOTOR_2_COLOR,
        label="M2 desired velocity",
    )

    velocity_axis.axhline(0.0, linewidth=1, alpha=0.5)
    velocity_axis.set_title(
        "Angular Velocity: Measured vs Desired",
        fontsize=10,
    )
    velocity_axis.set_xlabel("Time t (s)", fontsize=8)
    velocity_axis.set_ylabel("Velocity (degrees/s)", fontsize=8)
    velocity_axis.grid(True, alpha=0.3)
    velocity_axis.set_xlim(0.0, 1.0)
    velocity_axis.set_ylim(-30.0, 30.0)
    velocity_axis.legend(loc="upper right", fontsize=7, ncol=2)

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

        m1_velocity_history.clear()
        m1_target_velocity_history.clear()
        m2_velocity_history.clear()
        m2_target_velocity_history.clear()

        m1_u_history.clear()
        m2_u_history.clear()

        start_time = time.monotonic()

        for line in (
            m1_angle_line,
            m1_target_line,
            m2_angle_line,
            m2_target_line,
            m1_velocity_line,
            m1_target_velocity_line,
            m2_velocity_line,
            m2_target_velocity_line,
            m1_u_time_line,
            m2_u_time_line,
        ):
            line.set_data([], [])

        angle_axis.set_xlim(0.0, 1.0)
        velocity_axis.set_xlim(0.0, 1.0)
        velocity_axis.set_ylim(-30.0, 30.0)
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

        elif key in ["left", "a"]:
            # Both the left arrow and the A key request reverse/down movement.
            # A single-byte command is more dependable than forwarding an ANSI
            # arrow-key escape sequence through pyserial.
            send_serial_command("a")
            print("Manual reverse/down command sent to selected motor.")

        elif key in ["right", "d"]:
            # Both the right arrow and the D key request forward/up movement.
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
        m1_velocity = float(telemetry["m1_velocity"])
        m1_target_velocity = float(telemetry["m1_target_velocity"])

        m2_current = float(telemetry["m2_current"])
        m2_target = float(telemetry["m2_target"])
        m2_u = float(telemetry["m2_u_cmd"])
        m2_velocity = float(telemetry["m2_velocity"])
        m2_target_velocity = float(telemetry["m2_target_velocity"])

        time_history.append(elapsed_time)

        m1_angle_history.append(m1_current)
        m1_target_history.append(m1_target)
        m2_angle_history.append(m2_current)
        m2_target_history.append(m2_target)

        m1_velocity_history.append(m1_velocity)
        m1_target_velocity_history.append(m1_target_velocity)
        m2_velocity_history.append(m2_velocity)
        m2_target_velocity_history.append(m2_target_velocity)

        m1_u_history.append(m1_u)
        m2_u_history.append(m2_u)

        # Graph 1: current and target angles.
        m1_angle_line.set_data(time_history, m1_angle_history)
        m1_target_line.set_data(time_history, m1_target_history)
        m2_angle_line.set_data(time_history, m2_angle_history)
        m2_target_line.set_data(time_history, m2_target_history)

        # Graph 2: exact controller velocity tracking.
        m1_velocity_line.set_data(time_history, m1_velocity_history)
        m1_target_velocity_line.set_data(
            time_history,
            m1_target_velocity_history,
        )
        m2_velocity_line.set_data(time_history, m2_velocity_history)
        m2_target_velocity_line.set_data(
            time_history,
            m2_target_velocity_history,
        )

        # Graph 3: signed PWM command.
        m1_u_time_line.set_data(time_history, m1_u_history)
        m2_u_time_line.set_data(time_history, m2_u_history)

        if time_history:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 0.5)

            angle_axis.set_xlim(left_time, right_time)
            velocity_axis.set_xlim(left_time, right_time)
            u_time_axis.set_xlim(left_time, right_time)

        velocity_values = (
            list(m1_velocity_history)
            + list(m1_target_velocity_history)
            + list(m2_velocity_history)
            + list(m2_target_velocity_history)
        )
        if velocity_values:
            velocity_low = min(velocity_values)
            velocity_high = max(velocity_values)
            velocity_span = max(velocity_high - velocity_low, 20.0)
            velocity_center = 0.5 * (velocity_low + velocity_high)
            velocity_margin = 0.12 * velocity_span
            velocity_axis.set_ylim(
                velocity_center - 0.5 * velocity_span - velocity_margin,
                velocity_center + 0.5 * velocity_span + velocity_margin,
            )

        status_lines = [
            (
                "Keys: j select M1 | k select M2 | left/a right/d manual | "
                "0-9 target | x oscillation | s/space stop | r zero | "
                "g reset | q quit"
            ),
            (
                f"Selected: {telemetry['selected_motor']} | "
                f"M1 {telemetry['m1_mode']}: target "
                f"{float(telemetry['m1_target']):.1f}°, current "
                f"{float(telemetry['m1_current']):.1f}°, error "
                f"{float(telemetry['m1_error']):.1f}°, velocity "
                f"{float(telemetry['m1_velocity']):.2f}°/s, desired "
                f"{float(telemetry['m1_target_velocity']):.2f}°/s, PWM "
                f"{float(telemetry['m1_u_cmd']):.0f}, counts "
                f"{telemetry['m1_counts']}"
            ),
            (
                f"M2 {telemetry['m2_mode']}: target "
                f"{float(telemetry['m2_target']):.1f}°, current "
                f"{float(telemetry['m2_current']):.1f}°, error "
                f"{float(telemetry['m2_error']):.1f}°, velocity "
                f"{float(telemetry['m2_velocity']):.2f}°/s, desired "
                f"{float(telemetry['m2_target_velocity']):.2f}°/s, PWM "
                f"{float(telemetry['m2_u_cmd']):.0f}, counts "
                f"{telemetry['m2_counts']} | Velocity source: "
                f"{telemetry['velocity_source']} | Rejected spikes: "
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
            m1_velocity_line,
            m1_target_velocity_line,
            m2_velocity_line,
            m2_target_velocity_line,
            m1_u_time_line,
            m2_u_time_line,
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