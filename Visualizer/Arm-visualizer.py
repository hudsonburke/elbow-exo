#!/usr/bin/env python3
"""
Quaternion-only 3D arm visualizer for the Arduino BNO055 output.

Install:
    python -m pip install pyserial matplotlib numpy

Run:
    python arm_visualizer_quaternion_only.py

Important:
- Close PlatformIO Serial Monitor before running this.
- Only one program can use the COM port at a time.
- Press Z in the plot window to send "zero" to Arduino.
- Press R to reset the Python-side visualization memory.
- Press Q to quit.

This version uses quaternion values for the arm geometry.
Euler values are parsed only for human-readable status text.
"""

import re
import threading
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial


# =====================
# Configuration
# =====================

SERIAL_PORT = "COM4"
BAUD_RATE = 115200

UPPER_ARM_LENGTH = 0.30
FOREARM_LENGTH = 0.25

# Local direction of each arm segment in the IMU's own coordinate frame.
# If the visual bends in the wrong plane, try [0, 1, 0] or [0, 0, 1].
SEGMENT_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

# If the forearm points backward, set this to -1.0.
FOREARM_DIRECTION_SIGN = 1.0

MAX_DEBUG_LINES = 200


# =====================
# Regex patterns
# =====================

IMU_HEADER_RE = re.compile(r"^\s*IMU\s+(\d+)\s+Cal")
QUAT_RE = re.compile(
    r"Quat\(w,x,y,z\)\s*:\s*"
    r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*"
    r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*"
    r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*"
    r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
)
EULER_RE = re.compile(
    r"Euler\(H,R,P\)\s*:\s*"
    r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*"
    r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*"
    r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
)
JOINT_HEADER_RE = re.compile(r"^\s*Joint\s+(\d+)-(\d+)")


# =====================
# Runtime state
# =====================

line_queue = deque()
debug_lines = deque(maxlen=MAX_DEBUG_LINES)

stop_event = threading.Event()
ser_global = None

# Quaternions used for kinematics
latest_imu_quats = {}       # IMU index string -> np.array([w, x, y, z])
latest_joint_quats = {}     # parent IMU index string -> np.array([w, x, y, z])

# Euler values are display-only
latest_imu_eulers = {}      # IMU index string -> (heading, roll, pitch)
latest_joint_eulers = {}    # parent IMU index string -> (heading, roll, pitch)

parse_state = {
    "current_imu": None,
    "current_joint": None,
    "pending_quat": None,
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


def quat_multiply(q1, q2):
    """Hamilton product for quaternions stored as [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def rotate_vector_by_quat(v, q):
    """Rotate a 3D vector by quaternion q using q * v * q_conjugate."""
    q = normalize_quat(q)
    vq = np.array([0.0, v[0], v[1], v[2]])
    rotated = quat_multiply(quat_multiply(q, vq), quat_conjugate(q))
    return rotated[1:]


def quat_angle_deg(q):
    """Return the shortest rotation angle represented by q, in degrees."""
    q = normalize_quat(q)
    w = abs(np.clip(q[0], -1.0, 1.0))
    return np.degrees(2.0 * np.arccos(w))


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


def send_command(command):
    global ser_global

    if ser_global is None or not ser_global.is_open:
        print("Serial port is not open.")
        return

    try:
        ser_global.write((command + "\n").encode("utf-8"))
        print(f"Sent command to Arduino: {command}")
    except Exception as e:
        print(f"Failed to send command: {e}")


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
    imu_match = IMU_HEADER_RE.search(line)
    if imu_match:
        return "imu_header", {"imu_id": int(imu_match.group(1))}

    joint_match = JOINT_HEADER_RE.search(line)
    if joint_match:
        # Header format is Joint child-parent, e.g. Joint 1-0.
        child = int(joint_match.group(1))
        parent = int(joint_match.group(2))
        return "joint_header", {"child": child, "parent": parent}

    quat_match = QUAT_RE.search(line)
    if quat_match:
        return "quaternion", {
            "quat": np.array([
                float(quat_match.group(1)),
                float(quat_match.group(2)),
                float(quat_match.group(3)),
                float(quat_match.group(4)),
            ])
        }

    euler_match = EULER_RE.search(line)
    if euler_match:
        return "euler", {
            "euler": (
                float(euler_match.group(1)),
                float(euler_match.group(2)),
                float(euler_match.group(3)),
            )
        }

    if "==================" in line:
        return "separator", {}

    return None


def update_latest_data():
    """Process queued serial lines and update quaternion/euler dictionaries."""
    while line_queue:
        line = line_queue.popleft()
        result = parse_line(line)
        if result is None:
            continue

        line_type, data = result

        if line_type == "imu_header":
            parse_state["current_imu"] = data["imu_id"]
            parse_state["current_joint"] = None
            parse_state["pending_quat"] = None

        elif line_type == "joint_header":
            parse_state["current_joint"] = (data["child"], data["parent"])
            parse_state["current_imu"] = None
            parse_state["pending_quat"] = None

        elif line_type == "quaternion":
            parse_state["pending_quat"] = normalize_quat(data["quat"])

        elif line_type == "euler":
            q = parse_state["pending_quat"]
            if q is None:
                continue

            if parse_state["current_imu"] is not None:
                key = str(parse_state["current_imu"])
                latest_imu_quats[key] = q
                latest_imu_eulers[key] = data["euler"]

            elif parse_state["current_joint"] is not None:
                child, parent = parse_state["current_joint"]
                # Store Joint child-parent using the parent index.
                # Example: Joint 1-0 is the elbow joint after IMU 0.
                key = str(parent)
                latest_joint_quats[key] = q
                latest_joint_eulers[key] = data["euler"]

            parse_state["pending_quat"] = None
            parse_state["current_imu"] = None
            parse_state["current_joint"] = None

        elif line_type == "separator":
            parse_state["current_imu"] = None
            parse_state["current_joint"] = None
            parse_state["pending_quat"] = None


# =====================
# Kinematics
# =====================

def build_arm_positions():
    """
    Build 3D arm positions using quaternions only.

    Preferred path:
    - Upper arm direction = IMU 0 absolute quaternion rotating SEGMENT_AXIS_LOCAL.
    - Forearm direction = IMU 1 absolute quaternion rotating SEGMENT_AXIS_LOCAL.

    Fallback:
    - If IMU 1 is unavailable but Joint 1-0 is available, rotate the upper-arm
      direction by the Joint 1-0 relative quaternion.
    """
    shoulder = np.array([0.0, 0.0, 0.0])
    base_axis = SEGMENT_AXIS_LOCAL / np.linalg.norm(SEGMENT_AXIS_LOCAL)

    q0 = latest_imu_quats.get("0")
    q1 = latest_imu_quats.get("1")
    q_joint_0 = latest_joint_quats.get("0")

    if q0 is not None:
        upper_dir = rotate_vector_by_quat(base_axis, q0)
    else:
        upper_dir = base_axis.copy()

    if q1 is not None:
        forearm_dir = rotate_vector_by_quat(base_axis * FOREARM_DIRECTION_SIGN, q1)
    elif q_joint_0 is not None:
        forearm_dir = rotate_vector_by_quat(upper_dir * FOREARM_DIRECTION_SIGN, q_joint_0)
    else:
        forearm_dir = upper_dir.copy() * FOREARM_DIRECTION_SIGN

    # Normalize directions to prevent tiny numerical scaling drift.
    upper_dir = upper_dir / np.linalg.norm(upper_dir)
    forearm_dir = forearm_dir / np.linalg.norm(forearm_dir)

    elbow = shoulder + UPPER_ARM_LENGTH * upper_dir
    wrist = elbow + FOREARM_LENGTH * forearm_dir

    elbow_angle = quat_angle_deg(q_joint_0) if q_joint_0 is not None else 0.0

    return shoulder, elbow, wrist, elbow_angle


# =====================
# Main
# =====================

def main():
    global ser_global, latest_imu_quats, latest_joint_quats, latest_imu_eulers, latest_joint_eulers

    ser_global = open_serial(SERIAL_PORT, BAUD_RATE)
    if ser_global is None:
        return

    t = threading.Thread(target=serial_reader_thread, args=(ser_global,), daemon=True)
    t.start()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    total_len = UPPER_ARM_LENGTH + FOREARM_LENGTH
    lim = total_len + 0.1
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Quaternion-Only 3D Arm Visualizer | Z=zero, R=reset, Q=quit")

    shoulder_dot, = ax.plot([], [], [], "ko", markersize=6)
    elbow_dot, = ax.plot([], [], [], "ro", markersize=6)
    wrist_dot, = ax.plot([], [], [], "bo", markersize=6)
    upper_line, = ax.plot([], [], [], "r-", linewidth=3)
    fore_line, = ax.plot([], [], [], "b-", linewidth=3)

    text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, verticalalignment="top")

    def on_key(event):
        global latest_imu_quats, latest_joint_quats, latest_imu_eulers, latest_joint_eulers

        if event.key == "z":
            send_command("zero")
        elif event.key == "r":
            latest_imu_quats = {}
            latest_joint_quats = {}
            latest_imu_eulers = {}
            latest_joint_eulers = {}
            print("Reset Python-side visualizer state.")
        elif event.key == "q":
            stop_event.set()
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(frame):
        update_latest_data()

        shoulder, elbow, wrist, elbow_angle = build_arm_positions()

        shoulder_dot.set_data([shoulder[0]], [shoulder[1]])
        shoulder_dot.set_3d_properties([shoulder[2]])
        elbow_dot.set_data([elbow[0]], [elbow[1]])
        elbow_dot.set_3d_properties([elbow[2]])
        wrist_dot.set_data([wrist[0]], [wrist[1]])
        wrist_dot.set_3d_properties([wrist[2]])

        upper_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
        upper_line.set_3d_properties([shoulder[2], elbow[2]])
        fore_line.set_data([elbow[0], wrist[0]], [elbow[1], wrist[1]])
        fore_line.set_3d_properties([elbow[2], wrist[2]])

        imu0_euler = latest_imu_eulers.get("0")
        joint0_euler = latest_joint_eulers.get("0")

        status_text = "Kinematics: quaternion only\n"
        status_text += f"IMU quats: {len(latest_imu_quats)}\n"
        status_text += f"Joint quats: {len(latest_joint_quats)}\n"
        status_text += f"Elbow quat angle: {elbow_angle:.1f}\n"

        if imu0_euler is not None:
            h, r, p = imu0_euler
            status_text += f"\nIMU 0 Euler H,R,P: {h:.1f}, {r:.1f}, {p:.1f}"
        if joint0_euler is not None:
            h, r, p = joint0_euler
            status_text += f"\nJoint 1-0 Euler H,R,P: {h:.1f}, {r:.1f}, {p:.1f}"

        status_text += "\n\nZ=zero, R=reset, Q=quit"
        text.set_text(status_text)

        return shoulder_dot, elbow_dot, wrist_dot, upper_line, fore_line, text

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
