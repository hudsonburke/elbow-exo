#!/usr/bin/env python3

# Keyboard controls inside the plot window:
#   0-9        send target angle = digit * 10 degrees
#   left       manual motor reverse
#   right      manual motor forward
#   p          pause both motors, same as sending 's' in cpp file
#   o          oscillation mode
#   r          recalibrate/zero joint angle
#   c          clear Python-side stored telemetry
#   g          reset the joint angle graph
#   q          quit visualizer


import re
import threading
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial


# =====================
# Configuration
# =====================

SERIAL_PORT = "COM8"
BAUD_RATE = 9600

# Segment distances in meters. Tune these to match your actual mounting.
SHOULDER_TO_UPPER_IMU = 1
UPPER_IMU_TO_ELBOW = 5.25
ELBOW_TO_FOREARM_IMU = 3
FOREARM_IMU_TO_HAND = 1

# Tracking axis: your testing showed the IMU/arm motion behaves correctly
# when the arm segment is treated as the IMU's local +X axis.
# Do not change this just to change the visual rest direction.
IMU_SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

# Display-only offset: rotate the correctly tracked +X arm direction so that
# the zero/rest pose appears straight down along world -Z.
# This is a +90 degree rotation about world Y: +X -> -Z.
Q_DISPLAY_OFFSET = np.array([-0.7071067811865476, 0.0, 0.7071067811865476, 0.0])

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

Q_DISPLAY_OFFSET = quat_multiply(
    [-0.7071067811865476, 0.0, 0.0, 0.7071067811865476],
    Q_DISPLAY_OFFSET)

# Flip these if either visual segment points opposite of the real arm.
UPPER_DIRECTION_SIGN = -1.0
FOREARM_DIRECTION_SIGN = -1.0

MAX_DEBUG_LINES = 200


# =====================
# Regex patterns
# =====================

FLOAT_RE = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

Q_LABEL_RE = re.compile(
    rf"^\s*(qUpperZeroed|qForearmZeroed|qJointZeroed)\s*:\s*"
    rf"({FLOAT_RE}),\s*({FLOAT_RE}),\s*({FLOAT_RE}),\s*({FLOAT_RE})"
)

CURRENT_DEG_RE = re.compile(rf"CurrentDeg\s*:\s*({FLOAT_RE})")
TARGET_DEG_RE = re.compile(rf"TargetDeg\s*:\s*({FLOAT_RE})")
ERROR_DEG_RE = re.compile(rf"ErrorDeg\s*:\s*({FLOAT_RE})")
MODE_RE = re.compile(r"Mode\s*:\s*([A-Za-z]+)")
TELEMETRY_SEEN = False
COUNTS_RE = re.compile(r"M1Counts\s*:\s*(-?\d+)\s*\|\s*M2Counts\s*:\s*(-?\d+)")


# =====================
# Runtime state
# =====================

line_queue = deque()
debug_lines = deque(maxlen=MAX_DEBUG_LINES)
stop_event = threading.Event()
ser_global = None

latest = {
    "q_upper": np.array([1.0, 0.0, 0.0, 0.0]),
    "q_forearm": np.array([1.0, 0.0, 0.0, 0.0]),
    "q_joint": np.array([1.0, 0.0, 0.0, 0.0]),
    "current_deg": 0.0,
    "target_deg": 0.0,
    "error_deg": 0.0,
    "mode": "unknown",
    "m1_counts": 0,
    "m2_counts": 0,
    "lines_parsed": 0,
    "quat_lines_parsed": 0,
}


# =====================
# Quaternion math
# =====================

def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_conjugate(q):
    q = normalize_quat(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


# def quat_multiply(q1, q2):
#     w1, x1, y1, z1 = q1
#     w2, x2, y2, z2 = q2
#     return np.array([
#         w1*w2 - x1*x2 - y1*y2 - z1*z2,
#         w1*x2 + x1*w2 + y1*z2 - z1*y2,
#         w1*y2 - x1*z2 + y1*w2 + z1*x2,
#         w1*z2 + x1*y2 - y1*x2 + z1*w2,
#     ])


def rotate_vector_by_quat(v, q):
    q = normalize_quat(q)
    vq = np.array([0.0, v[0], v[1], v[2]])
    return quat_multiply(quat_multiply(q, vq), quat_conjugate(q))[1:]


def unit_vector(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.array([0.0, 0.0, -1.0])
    return v / n


def quat_angle_deg(q):
    q = normalize_quat(q)
    w = abs(np.clip(q[0], -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(w)))


# =====================
# Serial functions
# =====================

def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Opened serial port {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"Failed to open serial port {port}: {e}")
        return None


def send_serial(text, newline=True):
    if ser_global is None or not ser_global.is_open:
        print("Serial port is not open.")
        return

    payload = text + ("\n" if newline else "")
    try:
        ser_global.write(payload.encode("utf-8"))
        printable = text.encode("unicode_escape").decode("ascii")
        print(f"Sent: {printable}")
    except Exception as e:
        print(f"Failed to send serial command: {e}")


def serial_reader_thread(ser):
    try:
        while not stop_event.is_set():
            try:
                raw = ser.readline()
            except Exception:
                break

            if not raw:
                continue

            line = raw.decode(errors="ignore").strip()
            if line:
                line_queue.append(line)
                debug_lines.append(line)
    finally:
        try:
            ser.close()
        except Exception:
            pass


# =====================
# Parsing
# =====================

def parse_line(line):
    latest["lines_parsed"] += 1

    q_match = Q_LABEL_RE.search(line)
    if q_match:
        label = q_match.group(1)
        q = normalize_quat([
            float(q_match.group(2)),
            float(q_match.group(3)),
            float(q_match.group(4)),
            float(q_match.group(5)),
        ])
        if label == "qUpperZeroed":
            latest["q_upper"] = q
        elif label == "qForearmZeroed":
            latest["q_forearm"] = q
        elif label == "qJointZeroed":
            latest["q_joint"] = q
        latest["quat_lines_parsed"] += 1
        return

    current_match = CURRENT_DEG_RE.search(line)
    if current_match:
        latest["current_deg"] = float(current_match.group(1))

    target_match = TARGET_DEG_RE.search(line)
    if target_match:
        latest["target_deg"] = float(target_match.group(1))

    error_match = ERROR_DEG_RE.search(line)
    if error_match:
        latest["error_deg"] = float(error_match.group(1))

    mode_match = MODE_RE.search(line)
    if mode_match:
        latest["mode"] = mode_match.group(1)

    counts_match = COUNTS_RE.search(line)
    if counts_match:
        latest["m1_counts"] = int(counts_match.group(1))
        latest["m2_counts"] = int(counts_match.group(2))


def update_latest_data():
    while line_queue:
        parse_line(line_queue.popleft())


# =====================
# Kinematics
# =====================

def build_arm_positions():
    """Return shoulder, upper IMU, elbow, forearm IMU, hand positions."""
    shoulder = np.array([0.0, 0.0, 0.0])
    base_axis = unit_vector(IMU_SEGMENT_AXIS_LOCAL)

    # First apply the IMU's zeroed orientation for correct motion tracking.
    # Then apply a display-only offset so the zero pose points downward.
    q_upper_display = normalize_quat(quat_multiply(Q_DISPLAY_OFFSET, latest["q_upper"]))
    q_forearm_display = normalize_quat(quat_multiply(Q_DISPLAY_OFFSET, latest["q_forearm"]))

    upper_dir = rotate_vector_by_quat(base_axis * UPPER_DIRECTION_SIGN, q_upper_display)
    forearm_dir = rotate_vector_by_quat(base_axis * FOREARM_DIRECTION_SIGN, q_forearm_display)

    upper_dir = unit_vector(upper_dir)
    forearm_dir = unit_vector(forearm_dir)

    upper_imu = shoulder + SHOULDER_TO_UPPER_IMU * upper_dir
    elbow = shoulder + (SHOULDER_TO_UPPER_IMU + UPPER_IMU_TO_ELBOW) * upper_dir
    forearm_imu = elbow + ELBOW_TO_FOREARM_IMU * forearm_dir
    hand = elbow + (ELBOW_TO_FOREARM_IMU + FOREARM_IMU_TO_HAND) * forearm_dir

    return shoulder, upper_imu, elbow, forearm_imu, hand


def fmt_point(name, p):
    return f"{name}: ({p[0]: .3f}, {p[1]: .3f}, {p[2]: .3f})"


# =====================
# Main
# =====================

def main():
    global ser_global, latest

    ser_global = open_serial(SERIAL_PORT, BAUD_RATE)
    if ser_global is None:
        return

    reader = threading.Thread(target=serial_reader_thread, args=(ser_global,), daemon=True)
    reader.start()

    fig = plt.figure(figsize=(14, 7))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    angle_ax = fig.add_subplot(1, 2, 2)

    history_time = deque(maxlen=300)
    history_angle = deque(maxlen=300)
    history_start_time = time.monotonic()

    total_len = SHOULDER_TO_UPPER_IMU + UPPER_IMU_TO_ELBOW + ELBOW_TO_FOREARM_IMU + FOREARM_IMU_TO_HAND
    lim = total_len + 0.10
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("IMU Arm Visualizer + Motor Controller")

    # Initialize the plot with the default straight-down arm immediately.
    p0 = build_arm_positions()
    shoulder_dot, = ax.plot([p0[0][0]], [p0[0][1]], [p0[0][2]], "o", markersize=7, label="Shoulder")
    upper_imu_dot, = ax.plot([p0[1][0]], [p0[1][1]], [p0[1][2]], "^", markersize=6, label="Upper IMU")
    elbow_dot, = ax.plot([p0[2][0]], [p0[2][1]], [p0[2][2]], "o", markersize=7, label="Elbow")
    forearm_imu_dot, = ax.plot([p0[3][0]], [p0[3][1]], [p0[3][2]], "^", markersize=6, label="Forearm IMU")
    hand_dot, = ax.plot([p0[4][0]], [p0[4][1]], [p0[4][2]], "o", markersize=7, label="Hand")

    upper_line, = ax.plot([p0[0][0], p0[2][0]], [p0[0][1], p0[2][1]], [p0[0][2], p0[2][2]], linewidth=3)
    forearm_line, = ax.plot([p0[2][0], p0[4][0]], [p0[2][1], p0[4][1]], [p0[2][2], p0[4][2]], linewidth=3)
    ax.legend(loc="upper right")

    angle_line, = angle_ax.plot([], [], color="tab:blue", linewidth=2, label="Joint angle")
    angle_ax.set_title("Joint Angle vs Time")
    angle_ax.set_xlabel("Time (s)")
    angle_ax.set_ylabel("Angle (deg)")
    angle_ax.grid(True, alpha=0.3)
    angle_ax.set_ylim(-180, 180)
    angle_ax.legend(loc="upper right")

    text = fig.text(0.02, 0.01, "", transform=fig.transFigure, verticalalignment="bottom", fontsize=9)

    def on_key(event):
        global latest
        nonlocal history_start_time

        if event.key in [str(i) for i in range(10)]:
            send_serial(event.key)
        elif event.key == "left":
            send_serial("\x1b[D", newline=False)
        elif event.key == "right":
            send_serial("\x1b[C", newline=False)
        elif event.key in ("r"):
            send_serial("r")
        elif event.key == "p":
            send_serial("s")
        elif event.key == "m":
            send_serial("m")
        elif event.key == "o":
            send_serial("o")
        elif event.key == "c":
            latest = {
                "q_upper": np.array([1.0, 0.0, 0.0, 0.0]),
                "q_forearm": np.array([1.0, 0.0, 0.0, 0.0]),
                "q_joint": np.array([1.0, 0.0, 0.0, 0.0]),
                "current_deg": 0.0,
                "target_deg": 0.0,
                "error_deg": 0.0,
                "mode": "cleared",
                "m1_counts": 0,
                "m2_counts": 0,
                "lines_parsed": 0,
                "quat_lines_parsed": 0,
            }
            history_time.clear()
            history_angle.clear()
            history_start_time = time.monotonic()
            angle_line.set_data([], [])
            angle_ax.set_ylim(-180, 180)
            print("Cleared Python-side telemetry.")
        elif event.key == "g":
            history_time.clear()
            history_angle.clear()
            history_start_time = time.monotonic()
            angle_line.set_data([], [])
            angle_ax.set_xlim(0.0, 1.0)
            angle_ax.set_ylim(-180, 180)
            print("Reset joint angle graph.")
        elif event.key == "q":
            stop_event.set()
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        update_latest_data()

        shoulder, upper_imu, elbow, forearm_imu, hand = build_arm_positions()

        points = [shoulder, upper_imu, elbow, forearm_imu, hand]
        dots = [shoulder_dot, upper_imu_dot, elbow_dot, forearm_imu_dot, hand_dot]
        for dot, p in zip(dots, points):
            dot.set_data([p[0]], [p[1]])
            dot.set_3d_properties([p[2]])

        upper_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
        upper_line.set_3d_properties([shoulder[2], elbow[2]])

        forearm_line.set_data([elbow[0], hand[0]], [elbow[1], hand[1]])
        forearm_line.set_3d_properties([elbow[2], hand[2]])

        joint_display_deg = latest["current_deg"]
        if latest["quat_lines_parsed"] > 0 and abs(joint_display_deg) < 1e-9:
            joint_display_deg = quat_angle_deg(latest["q_joint"])

        history_time.append(time.monotonic() - history_start_time)
        history_angle.append(joint_display_deg)
        angle_line.set_data(list(history_time), list(history_angle))
        if history_angle:
            angle_ax.set_xlim(0.0, max(1.0, history_time[-1] + 1.0))
            min_angle = min(history_angle) - 5
            max_angle = max(history_angle) + 5
            angle_ax.set_ylim(min_angle, max_angle)

        status = [
            "Keys: 0-9 target | left/right manual | p pause | r recalibrate | m menu | c clear | g reset graph | q quit",
            f"Target: {latest['target_deg']:.2f} deg | Joint: {joint_display_deg:.2f} deg | Error: {latest['error_deg']:.2f} deg",
            fmt_point("Shoulder", shoulder),
            fmt_point("Upper IMU", upper_imu),
            fmt_point("Elbow", elbow),
            fmt_point("Forearm IMU", forearm_imu),
            fmt_point("Hand", hand),
        ]
        text.set_text("\n".join(status))

        return (*dots, upper_line, forearm_line, angle_line, text)

    # Keep a reference to the animation. Without this, Matplotlib can garbage-collect
    # the animation object and the update function may never run.
    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    fig._arm_visualizer_animation = ani

    try:
        plt.show()
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
