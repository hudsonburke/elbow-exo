#!/usr/bin/env python3
"""

- Press Z in the plot window to send "zero" to Arduino.
- Press R to reset the Python-side visualization memory.
- Press Q to quit.


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

# Open the serial port that communicates with the Arduino.
# If the port cannot be opened, this returns None.
def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Opened serial port {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"Failed to open serial port {port}: {e}")
        return None


# Send a text command to the Arduino. This is used by the key handler.
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


# Read lines from serial in a separate thread so the plot stays responsive.
# Each received line is stored in `line_queue` for later parsing.
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
    """Read one serial line and classify it by type.

    The Arduino output includes sections for IMU orientation and joint
    relative rotation. This function finds the line type and returns a
    simplified structure for later processing.
    """
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
            # Start reading a new IMU block.
            parse_state["current_imu"] = data["imu_id"]
            parse_state["current_joint"] = None
            parse_state["pending_quat"] = None

        elif line_type == "joint_header":
            # Start reading a new joint block.
            parse_state["current_joint"] = (data["child"], data["parent"])
            parse_state["current_imu"] = None
            parse_state["pending_quat"] = None

        elif line_type == "quaternion":
            # Store the quaternion until we get the matching Euler line.
            parse_state["pending_quat"] = normalize_quat(data["quat"])

        elif line_type == "euler":
            # Once we have both quaternion and Euler for the same block,
            # store them together for display or kinematics.
            q = parse_state["pending_quat"]
            if q is None:
                continue

            if parse_state["current_imu"] is not None:
                key = str(parse_state["current_imu"])
                latest_imu_quats[key] = q
                latest_imu_eulers[key] = data["euler"]

            elif parse_state["current_joint"] is not None:
                child, parent = parse_state["current_joint"]
                # Use the parent IMU index as the joint key.
                key = str(parent)
                latest_joint_quats[key] = q
                latest_joint_eulers[key] = data["euler"]

            parse_state["pending_quat"] = None
            parse_state["current_imu"] = None
            parse_state["current_joint"] = None

        elif line_type == "separator":
            # Reset parsing state at the end of an output block.
            parse_state["current_imu"] = None
            parse_state["current_joint"] = None
            parse_state["pending_quat"] = None


# =====================
# Kinematics
# =====================

def build_arm_positions():
    """Build the arm joint positions from the latest quaternion data."""
    # The shoulder is fixed at the origin for this visualization.
    shoulder = np.array([0.0, 0.0, 0.0])

    # The local segment axis is the direction of the arm segment in the IMU's
    # own coordinate frame before any rotation is applied.
    base_axis = SEGMENT_AXIS_LOCAL / np.linalg.norm(SEGMENT_AXIS_LOCAL)

    q0 = latest_imu_quats.get("0")
    q1 = latest_imu_quats.get("1")
    q_joint_0 = latest_joint_quats.get("0")

    if q0 is not None:
        # Rotate the base axis by IMU 0 to get the upper-arm direction.
        upper_dir = rotate_vector_by_quat(base_axis, q0)
    else:
        upper_dir = base_axis.copy()

    if q1 is not None:
        # Use IMU 1 absolute orientation for the forearm direction.
        forearm_dir = rotate_vector_by_quat(base_axis * FOREARM_DIRECTION_SIGN, q1)
    elif q_joint_0 is not None:
        # Fallback: use the relative joint quaternion if IMU 1 is not available.
        forearm_dir = rotate_vector_by_quat(upper_dir * FOREARM_DIRECTION_SIGN, q_joint_0)
    else:
        # If no quaternion data exists, point the forearm in the same plane.
        forearm_dir = upper_dir.copy() * FOREARM_DIRECTION_SIGN

    # Normalize the direction vectors so the visual lengths are stable.
    upper_dir = upper_dir / np.linalg.norm(upper_dir)
    forearm_dir = forearm_dir / np.linalg.norm(forearm_dir)

    # Calculate world positions for the elbow and wrist.
    elbow = shoulder + UPPER_ARM_LENGTH * upper_dir
    wrist = elbow + FOREARM_LENGTH * forearm_dir

    elbow_angle = quat_angle_deg(q_joint_0) if q_joint_0 is not None else 0.0

    return shoulder, elbow, wrist, elbow_angle


# =====================
# Main
# =====================

def main():
    """Start the Python visualizer and run the animation loop."""
    global ser_global, latest_imu_quats, latest_joint_quats, latest_imu_eulers, latest_joint_eulers

    # Open the serial connection first.
    ser_global = open_serial(SERIAL_PORT, BAUD_RATE)
    if ser_global is None:
        return

    # Read serial lines in a background thread so matplotlib stays responsive.
    t = threading.Thread(target=serial_reader_thread, args=(ser_global,), daemon=True)
    t.start()

    # Create a 3D matplotlib figure for the arm.
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

    # Plot objects for the joints and bones.
    shoulder_dot, = ax.plot([], [], [], "ko", markersize=6)
    elbow_dot, = ax.plot([], [], [], "ro", markersize=6)
    wrist_dot, = ax.plot([], [], [], "bo", markersize=6)
    upper_line, = ax.plot([], [], [], "r-", linewidth=3)
    fore_line, = ax.plot([], [], [], "b-", linewidth=3)

    # Place the status text below the 3D axes to avoid overlapping with the visualization.
    text = fig.text(0.02, 0.01, "", transform=fig.transFigure, verticalalignment="bottom", fontsize=9)

    def on_key(event):
        """Handle keyboard controls from the plot window."""
        global latest_imu_quats, latest_joint_quats, latest_imu_eulers, latest_joint_eulers

        if event.key == "z":
            # Send a zero command to the Arduino so it captures a reference orientation.
            send_command("z")
        elif event.key == "r":
            # Clear all stored data in Python.
            latest_imu_quats = {}
            latest_joint_quats = {}
            latest_imu_eulers = {}
            latest_joint_eulers = {}
            print("Reset Python-side visualizer state.")
        elif event.key == "q":
            # Quit the visualizer cleanly.
            stop_event.set()
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(frame):
        """Update the arm pose and status text every animation frame."""
        update_latest_data()

        shoulder, elbow, wrist, elbow_angle = build_arm_positions()

        # Update the joint marker positions.
        shoulder_dot.set_data([shoulder[0]], [shoulder[1]])
        shoulder_dot.set_3d_properties([shoulder[2]])
        elbow_dot.set_data([elbow[0]], [elbow[1]])
        elbow_dot.set_3d_properties([elbow[2]])
        wrist_dot.set_data([wrist[0]], [wrist[1]])
        wrist_dot.set_3d_properties([wrist[2]])

        # Update the bone line segments.
        upper_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
        upper_line.set_3d_properties([shoulder[2], elbow[2]])
        fore_line.set_data([elbow[0], wrist[0]], [elbow[1], wrist[1]])
        fore_line.set_3d_properties([elbow[2], wrist[2]])

        imu0_euler = latest_imu_eulers.get("0")
        imu1_euler = latest_imu_eulers.get("1")
        joint0_euler = latest_joint_eulers.get("0")

        # Build status text shown in the figure.
        status_text = f"IMU quats: {len(latest_imu_quats)}\n"
        status_text += f"Joint quats: {len(latest_joint_quats)}\n"
        status_text += f"Elbow quat angle: {elbow_angle:.1f}\n"
        status_text += f"Elbow position: ({elbow[0]:.3f}, {elbow[1]:.3f}, {elbow[2]:.3f})\n"
        status_text += f"Wrist position: ({wrist[0]:.3f}, {wrist[1]:.3f}, {wrist[2]:.3f})\n"

        if imu0_euler is not None:
            h, r, p = imu0_euler
            status_text += f"\nIMU 0 Euler H,R,P: {h:.1f}, {r:.1f}, {p:.1f}"
        if imu1_euler is not None:
            h, r, p = imu1_euler
            status_text += f"\nIMU 1 Euler H,R,P: {h:.1f}, {r:.1f}, {p:.1f}"

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
