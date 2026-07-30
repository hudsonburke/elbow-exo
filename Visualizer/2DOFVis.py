# #!/usr/bin/env python3

# Keyboard controls inside the plot window:
#   c          clear plotted telemetry
#   g          reset both graphs
#   q          quit visualizer


import re
import threading
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# =====================
# Configuration
# =====================

SERIAL_PORT = "COM9"
BAUD_RATE = 9600

# Segment lengths. These values appear to be inches in the current setup.
SHOULDER_TO_UPPER_IMU = 5
UPPER_IMU_TO_ELBOW = 5
ELBOW_TO_FOREARM_IMU = 7
FOREARM_IMU_TO_HAND = 4

# Tracking axis: your testing showed the IMU/arm motion behaves correctly
# when the arm segment is treated as the IMU's local +X axis.
# Do not change this just to change the visual rest direction.
IMU_SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

# Display-only offset: Used to set the "Zero Position" 
# to straight down, this section adjusts the world frame
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
# This is a 90 deg rotation about the new world z axis
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
    rf"^\s*(Upper\s+Quat|Lower\s+Quat)\s*:\s*:?\s*"
    rf"({FLOAT_RE}),\s*({FLOAT_RE}),\s*({FLOAT_RE}),\s*({FLOAT_RE})"
)
ELBOW_FLEX_RE = re.compile(rf"ElbowFlexDeg\s*:\s*({FLOAT_RE})")
FOREARM_PRON_RE = re.compile(rf"ForearmPronDeg\s*:\s*({FLOAT_RE})")


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
    "elbow_flex_deg": 0.0,
    "forearm_pron_deg": 0.0,
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


# This function was moved up to rotate world frame
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


def send_serial(text):
    if ser_global is None or not ser_global.is_open:
        print("Serial port is not open.")
        return

    try:
        ser_global.write((text + "\n").encode("utf-8"))
        print(f"Sent: {text}")
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
        if label == "Upper Quat":
            latest["q_upper"] = q
        elif label == "Lower Quat":
            latest["q_forearm"] = q
        latest["quat_lines_parsed"] += 1
        return

    elbow_match = ELBOW_FLEX_RE.search(line)
    if elbow_match:
        latest["elbow_flex_deg"] = float(elbow_match.group(1))

    forearm_match = FOREARM_PRON_RE.search(line)
    if forearm_match:
        latest["forearm_pron_deg"] = float(forearm_match.group(1))


def update_latest_data():
    while line_queue:
        parse_line(line_queue.popleft())


# =====================
# Kinematics
# =====================

def build_arm_positions():
    """Return arm points and rotation-aware width directions for both segments."""
    shoulder = np.array([0.0, 0.0, 0.0])
    base_axis = unit_vector(IMU_SEGMENT_AXIS_LOCAL)

    # First apply the IMU's zeroed orientation for correct motion tracking.
    # Then apply a display-only offset so the zero pose points downward.
    q_upper_display = normalize_quat(quat_multiply(Q_DISPLAY_OFFSET, latest["q_upper"]))
    q_forearm_display = normalize_quat(quat_multiply(Q_DISPLAY_OFFSET, latest["q_forearm"]))

    upper_dir = rotate_vector_by_quat(base_axis * UPPER_DIRECTION_SIGN, q_upper_display)
    forearm_dir = rotate_vector_by_quat(base_axis * FOREARM_DIRECTION_SIGN, q_forearm_display)
    upper_width_dir = rotate_vector_by_quat(np.array([0.0, 1.0, 0.0]), q_upper_display)
    forearm_width_dir = rotate_vector_by_quat(np.array([0.0, 1.0, 0.0]), q_forearm_display)

    upper_dir = unit_vector(upper_dir)
    forearm_dir = unit_vector(forearm_dir)
    upper_width_dir = unit_vector(upper_width_dir)
    forearm_width_dir = unit_vector(forearm_width_dir)

    upper_imu = shoulder + SHOULDER_TO_UPPER_IMU * upper_dir
    elbow = shoulder + (SHOULDER_TO_UPPER_IMU + UPPER_IMU_TO_ELBOW) * upper_dir
    forearm_imu = elbow + ELBOW_TO_FOREARM_IMU * forearm_dir
    hand = elbow + (ELBOW_TO_FOREARM_IMU + FOREARM_IMU_TO_HAND) * forearm_dir

    return shoulder, upper_imu, elbow, forearm_imu, hand, upper_width_dir, forearm_width_dir


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

    fig = plt.figure(figsize=(19, 7))
    fig.subplots_adjust(
    left=0.04,
    right=0.98,
    bottom=0.18,
    top=0.95,
    wspace=0.5
)
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    elbow_ax = fig.add_subplot(1, 3, 2)
    rotation_ax = fig.add_subplot(1, 3, 3)

    info_ax = fig.add_axes([0.04, 0.05, 0.92, 0.08])
    info_ax.axis("off")
    info_text = info_ax.text(
        0.02,
        0.5,
        "",
        ha="left",
        va="center",
        fontsize=14,
        family="monospace",
    )

    history_time = deque(maxlen=300)
    history_elbow_flexion = deque(maxlen=300)
    history_forearm_rotation = deque(maxlen=300)
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
    forearm_line, = ax.plot([p0[2][0], p0[4][0]], [p0[2][1], p0[4][1]], [p0[2][2], p0[4][2]], linewidth=1)
    upper_arm_plane = Poly3DCollection([], alpha=0.45, facecolor="tab:blue", edgecolor="tab:blue")
    forearm_plane = Poly3DCollection([], alpha=0.65, facecolor="tab:orange", edgecolor="tab:red")
    ax.add_collection3d(upper_arm_plane)
    ax.add_collection3d(forearm_plane)
    ax.legend(loc="upper right")

    elbow_line, = elbow_ax.plot([], [], color="tab:blue", linewidth=2)
    elbow_ax.set_title("Elbow Flexion vs Time")
    elbow_ax.set_xlabel("Time (s)")
    elbow_ax.set_ylabel("Flexion (deg)")
    elbow_ax.grid(True, alpha=0.3)
    elbow_ax.set_ylim(-5, 5)

    rotation_line, = rotation_ax.plot([], [], color="tab:orange", linewidth=2)
    rotation_ax.set_title("Forearm Rotation vs Time")
    rotation_ax.set_xlabel("Time (s)")
    rotation_ax.set_ylabel("Pronation (deg)")
    rotation_ax.grid(True, alpha=0.3)
    rotation_ax.set_ylim(-5, 5)

    # c clears telemetry, g clears graphs, r resets imus, q quits
    def on_key(event):
        nonlocal history_start_time

        if event.key == "c":
            history_time.clear()
            history_elbow_flexion.clear()
            history_forearm_rotation.clear()
            history_start_time = time.monotonic()
            elbow_line.set_data([], [])
            rotation_line.set_data([], [])
        elif event.key == "g":
            history_time.clear()
            history_elbow_flexion.clear()
            history_forearm_rotation.clear()
            history_start_time = time.monotonic()
            elbow_line.set_data([], [])
            rotation_line.set_data([], [])
        elif event.key == "r":
            send_serial("r")
        elif event.key == "q":
            stop_event.set()
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        update_latest_data()

        shoulder, upper_imu, elbow, forearm_imu, hand, upper_width_dir, forearm_width_dir = build_arm_positions()

        points = [shoulder, upper_imu, elbow, forearm_imu, hand]
        dots = [shoulder_dot, upper_imu_dot, elbow_dot, forearm_imu_dot, hand_dot]
        for dot, p in zip(dots, points):
            dot.set_data([p[0]], [p[1]])
            dot.set_3d_properties([p[2]])

        upper_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
        upper_line.set_3d_properties([shoulder[2], elbow[2]])

        forearm_line.set_data([elbow[0], hand[0]], [elbow[1], hand[1]])
        forearm_line.set_3d_properties([elbow[2], hand[2]])
        # change to adjust arm plane size, currently 2.5 inches wide
        half_width = 1.25
        upper_plane_corners = [
            shoulder - half_width * upper_width_dir,
            shoulder + half_width * upper_width_dir,
            elbow + half_width * upper_width_dir,
            elbow - half_width * upper_width_dir,
        ]
        forearm_plane_corners = [
            elbow - half_width * forearm_width_dir,
            elbow + half_width * forearm_width_dir,
            hand + half_width * forearm_width_dir,
            hand - half_width * forearm_width_dir,
        ]
        upper_arm_plane.set_verts([upper_plane_corners])
        forearm_plane.set_verts([forearm_plane_corners])

        history_time.append(time.monotonic() - history_start_time)
        history_elbow_flexion.append(latest["elbow_flex_deg"])
        history_forearm_rotation.append(latest["forearm_pron_deg"])

        info_text.set_text(
            f"Elbow Flexion: {latest['elbow_flex_deg']:.2f} deg\n"
            f"Forearm Rotation: {latest['forearm_pron_deg']:.2f} deg"
        )

        elbow_line.set_data(list(history_time), list(history_elbow_flexion))
        rotation_line.set_data(list(history_time), list(history_forearm_rotation))

        plot_xlim = max(1.0, history_time[-1] + 1.0)
        elbow_ax.set_xlim(0.0, plot_xlim)
        rotation_ax.set_xlim(0.0, plot_xlim)

        max_angle = max(
            5.0,
            max(abs(value) for value in history_elbow_flexion),
            max(abs(value) for value in history_forearm_rotation),
        )
        angle_limit = max_angle * 1.1
        elbow_ax.set_ylim(-angle_limit, angle_limit)
        rotation_ax.set_ylim(-angle_limit, angle_limit)

        return (
            *dots,
            upper_line,
            forearm_line,
            upper_arm_plane,
            forearm_plane,
            elbow_line,
            rotation_line,
        )

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