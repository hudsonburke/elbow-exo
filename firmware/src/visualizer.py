#!/usr/bin/env python3

# Keyboard controls inside the plot window:
#   0-9        send target angle = digit * 10 degrees
#   left       manual motor reverse
#   right      manual motor forward
#   p          pause both motors, same as sending 's' in C++ code
#   r          recalibrate/zero joint angle
#   c          clear Python-side stored telemetry
#   g          reset the graphs
#   q          quit visualizer

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

SERIAL_PORT = "COM8"
BAUD_RATE = 115200

# Window and layout settings
WINDOW_SIZE = (14, 10)

# Increase this number to make the 3D visualizer bigger.
# Decrease this number to make the 3D visualizer smaller.
ARM_PANEL_WIDTH = 1.1
GRAPH_PANEL_WIDTH = 0.9

HISTORY_POINTS = 300
ANIMATION_INTERVAL_MS = 35

# Segment distances. Tune these to match your real arm/exoskeleton setup.
SHOULDER_TO_UPPER_IMU = 1.0
UPPER_IMU_TO_ELBOW = 5.25
ELBOW_TO_FOREARM_IMU = 3.0
FOREARM_IMU_TO_HAND = 1.0

# IMU local axis used to point along each arm segment.
IMU_SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

# Flip these if the visual arm points opposite from the real arm.
UPPER_ARM_DIRECTION_SIGN = -1.0
FOREARM_DIRECTION_SIGN = -1.0

# Optional debug printing.
PRINT_RAW_SERIAL = False


# =====================================================
# Quaternion math
# =====================================================

def multiply_quaternions(q1, q2):
    """Multiply two quaternions.

    Quaternion format:
        [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


# Display-only offset.
# This rotates the visual zero/rest pose so the arm appears straight down.
DISPLAY_OFFSET_BASE = np.array([-0.7071067811865476, 0.0, 0.7071067811865476, 0.0])
DISPLAY_OFFSET_EXTRA = np.array([-0.7071067811865476, 0.0, 0.0, 0.7071067811865476])

DISPLAY_ROTATION = multiply_quaternions(DISPLAY_OFFSET_EXTRA, DISPLAY_OFFSET_BASE)


def normalize_quaternion(q):
    """Return a normalized quaternion."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)

    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])

    return q / norm


def conjugate_quaternion(q):
    """Return the conjugate of a quaternion."""
    q = normalize_quaternion(q)

    return np.array([q[0], -q[1], -q[2], -q[3]])


def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a 3D vector using a quaternion."""
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
    """Return a unit vector."""
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)

    if norm < 1e-12:
        return np.array([0.0, 0.0, -1.0])

    return vector / norm


def quaternion_angle_degrees(quaternion):
    """Convert quaternion rotation amount to degrees."""
    quaternion = normalize_quaternion(quaternion)

    w = abs(np.clip(quaternion[0], -1.0, 1.0))

    return float(np.degrees(2.0 * np.arccos(w)))


# =====================================================
# Regex patterns for serial parsing
# =====================================================

FLOAT_PATTERN = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

QUATERNION_LINE_PATTERN = re.compile(
    rf"^\s*(qUpperZeroed|qForearmZeroed|qJointZeroed)\s*:\s*"
    rf"({FLOAT_PATTERN}),\s*({FLOAT_PATTERN}),\s*({FLOAT_PATTERN}),\s*({FLOAT_PATTERN})"
)

CURRENT_ANGLE_PATTERN = re.compile(rf"CurrentDeg\s*:\s*({FLOAT_PATTERN})")
TARGET_ANGLE_PATTERN = re.compile(rf"TargetDeg\s*:\s*({FLOAT_PATTERN})")
ERROR_ANGLE_PATTERN = re.compile(rf"ErrorDeg\s*:\s*({FLOAT_PATTERN})")

PWM_NORM_PATTERN = re.compile(rf"PWMNorm\s*:\s*({FLOAT_PATTERN})")
PWM_PATTERN = re.compile(r"PWM\s*:\s*(-?\d+)")

MODE_PATTERN = re.compile(r"Mode\s*:\s*([A-Za-z]+)")
COUNTS_PATTERN = re.compile(r"M1Counts\s*:\s*(-?\d+)\s*\|\s*M2Counts\s*:\s*(-?\d+)")


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

    "pwm_norm": 0.0,
    "pwm": 0,

    "mode": "unknown",
    "motor_1_counts": 0,
    "motor_2_counts": 0,

    "lines_read": 0,
    "quaternion_lines_read": 0,
}


# =====================================================
# Serial communication
# =====================================================

def open_serial_port(port, baud_rate):
    """Open the serial port used by the Teensy/Arduino."""
    try:
        connection = serial.Serial(port, baud_rate, timeout=0.1)
        print(f"Opened serial port {port} @ {baud_rate}")
        return connection

    except Exception as error:
        print(f"Failed to open serial port {port}: {error}")
        return None


def send_serial_command(text, newline=True):
    """Send a command to the Teensy/Arduino."""
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
    """Continuously read serial lines in the background."""
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
    """Parse one line of serial data from the Teensy/Arduino."""
    telemetry["lines_read"] += 1

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

    mode_match = MODE_PATTERN.search(line)

    if mode_match:
        telemetry["mode"] = mode_match.group(1)

    counts_match = COUNTS_PATTERN.search(line)

    if counts_match:
        telemetry["motor_1_counts"] = int(counts_match.group(1))
        telemetry["motor_2_counts"] = int(counts_match.group(2))


def process_serial_queue():
    """Process all waiting serial lines."""
    while serial_line_queue:
        parse_serial_line(serial_line_queue.popleft())


# =====================================================
# Arm position calculation
# =====================================================

def calculate_arm_positions():
    """Calculate shoulder, elbow, IMU, and hand positions for the 3D plot."""
    shoulder = np.array([0.0, 0.0, 0.0])
    base_axis = normalize_vector(IMU_SEGMENT_AXIS_LOCAL)

    upper_display_quaternion = normalize_quaternion(
        multiply_quaternions(DISPLAY_ROTATION, telemetry["upper_quaternion"])
    )

    forearm_display_quaternion = normalize_quaternion(
        multiply_quaternions(DISPLAY_ROTATION, telemetry["forearm_quaternion"])
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
    """Format a 3D point for the status text."""
    return f"{name}: ({point[0]: .3f}, {point[1]: .3f}, {point[2]: .3f})"


def reset_telemetry():
    """Reset Python-side telemetry only."""
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

        "mode": "cleared",
        "motor_1_counts": 0,
        "motor_2_counts": 0,

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

    # 3 rows:
    #   Right top    = joint angle vs time
    #   Right middle = normalized PWM vs time
    #   Right bottom = actual PWM vs error
    grid = figure.add_gridspec(
        3,
        2,
        width_ratios=[ARM_PANEL_WIDTH, GRAPH_PANEL_WIDTH],
        height_ratios=[1.0, 1.0, 1.0],
        wspace=0.35,
        hspace=0.50
    )

    arm_axis = figure.add_subplot(grid[:, 0], projection="3d")
    angle_axis = figure.add_subplot(grid[0, 1])
    pwm_axis = figure.add_subplot(grid[1, 1])
    pwm_error_axis = figure.add_subplot(grid[2, 1])

    figure.subplots_adjust(
        left=0.04,
        right=0.97,
        top=0.96,
        bottom=0.24
    )

    time_history = deque(maxlen=HISTORY_POINTS)
    angle_history = deque(maxlen=HISTORY_POINTS)
    pwm_norm_history = deque(maxlen=HISTORY_POINTS)

    # Third graph history
    error_history = deque(maxlen=HISTORY_POINTS)
    pwm_history = deque(maxlen=HISTORY_POINTS)

    start_time = time.monotonic()

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

    arm_axis.set_xlabel("X", fontsize=9)
    arm_axis.set_ylabel("Y", fontsize=9)
    arm_axis.set_zlabel("Z", fontsize=9)
    arm_axis.set_title("Arm Visualizer", fontsize=12, pad=8)
    arm_axis.tick_params(axis="both", which="major", labelsize=8)

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

    # Joint angle graph
    angle_line, = angle_axis.plot([], [], linewidth=2, label="Joint angle")
    angle_axis.set_title("Joint Angle vs Time", fontsize=11)
    angle_axis.set_xlabel("Time (s)", fontsize=9)
    angle_axis.set_ylabel("Angle (deg)", fontsize=9)
    angle_axis.grid(True, alpha=0.3)
    angle_axis.set_ylim(-180, 180)
    angle_axis.legend(loc="upper right", fontsize=8)

    # PWMNorm vs time graph
    pwm_norm_line, = pwm_axis.plot([], [], linewidth=2, label="PWMNorm")
    pwm_axis.set_title("Normalized PWM Signal", fontsize=11)
    pwm_axis.set_xlabel("Time (s)", fontsize=9)
    pwm_axis.set_ylabel("Normalized PWM", fontsize=9)
    pwm_axis.grid(True, alpha=0.3)
    pwm_axis.set_ylim(-0.05, 1.05)
    pwm_axis.legend(loc="upper right", fontsize=8)

    # Actual PWM vs error graph
    pwm_error_line, = pwm_error_axis.plot(
        [],
        [],
        linewidth=2,
        marker=".",
        markersize=3,
        label="PWM vs Error"
    )

    pwm_error_axis.set_title("Actual PWM vs Error e(t)", fontsize=11)
    pwm_error_axis.set_xlabel("Error e(t) (deg)", fontsize=9)
    pwm_error_axis.set_ylabel("PWM value", fontsize=9)
    pwm_error_axis.grid(True, alpha=0.3)
    pwm_error_axis.set_xlim(-1.0, 1.0)
    pwm_error_axis.set_ylim(-5, 260)
    pwm_error_axis.legend(loc="upper right", fontsize=8)

    status_text = figure.text(
        0.02,
        0.01,
        "",
        transform=figure.transFigure,
        verticalalignment="bottom",
        fontsize=9
    )

    def reset_plot_history():
        """Clear graph history and restart the time axis."""
        nonlocal start_time

        time_history.clear()
        angle_history.clear()
        pwm_norm_history.clear()
        error_history.clear()
        pwm_history.clear()

        start_time = time.monotonic()

        angle_line.set_data([], [])
        angle_axis.set_xlim(0.0, 1.0)
        angle_axis.set_ylim(-180, 180)

        pwm_norm_line.set_data([], [])
        pwm_axis.set_xlim(0.0, 1.0)
        pwm_axis.set_ylim(-0.05, 1.05)

        pwm_error_line.set_data([], [])
        pwm_error_axis.set_xlim(-1.0, 1.0)
        pwm_error_axis.set_ylim(-5, 260)

    def handle_key_press(event):
        """Send keyboard commands to the Teensy/Arduino."""
        if event.key in [str(i) for i in range(10)]:
            send_serial_command(event.key)

        elif event.key == "left":
            send_serial_command("\x1b[D", newline=False)

        elif event.key == "right":
            send_serial_command("\x1b[C", newline=False)

        elif event.key == "r":
            send_serial_command("r")

        elif event.key == "space":
            send_serial_command("s")

        elif event.key == "m":
            send_serial_command("m")

        elif event.key == "c":
            reset_telemetry()
            reset_plot_history()
            print("Cleared Python-side telemetry.")

        elif event.key == "g":
            reset_plot_history()
            print("Reset graphs.")

        elif event.key == "q":
            stop_requested.set()
            plt.close(figure)

    figure.canvas.mpl_connect("key_press_event", handle_key_press)

    def update_plot(_frame):
        """Update the 3D arm and graphs."""
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
            joint_angle = quaternion_angle_degrees(telemetry["joint_quaternion"])

        current_time = time.monotonic() - start_time

        time_history.append(current_time)
        angle_history.append(joint_angle)
        pwm_norm_history.append(telemetry["pwm_norm"])

        error_history.append(telemetry["error_angle"])
        pwm_history.append(telemetry["pwm"])

        # Update joint angle graph
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

        # Update PWMNorm vs time graph
        pwm_norm_line.set_data(list(time_history), list(pwm_norm_history))

        if pwm_norm_history:
            left_time = time_history[0]
            right_time = max(left_time + 1.0, time_history[-1] + 1.0)

            pwm_axis.set_xlim(left_time, right_time)
            pwm_axis.set_ylim(-0.05, 1.05)

        # Update actual PWM vs error graph
        pwm_error_line.set_data(list(error_history), list(pwm_history))

        if error_history:
            min_error = min(error_history) - 2.0
            max_error = max(error_history) + 2.0

            if abs(max_error - min_error) < 1.0:
                min_error -= 1.0
                max_error += 1.0

            pwm_error_axis.set_xlim(min_error, max_error)
            pwm_error_axis.set_ylim(-5, 260)

        status_lines = [
            "Keys: 0-9 target | left/right manual | p pause | r recalibrate | m menu | c clear | g reset graphs | q quit",
            (
                f"Target: {telemetry['target_angle']:.2f} deg | "
                f"Joint: {joint_angle:.2f} deg | "
                f"Error: {telemetry['error_angle']:.2f} deg | "
                f"PWMNorm: {telemetry['pwm_norm']:.3f} | "
                f"PWM: {telemetry['pwm']} | "
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
            pwm_error_line,
            status_text,
        )

    arm_animation = animation.FuncAnimation(
        figure,
        update_plot,
        interval=ANIMATION_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    # Keep a reference so matplotlib does not garbage-collect the animation.
    figure._arm_visualizer_animation = arm_animation

    try:
        plt.show()

    finally:
        stop_requested.set()


if __name__ == "__main__":
    main()