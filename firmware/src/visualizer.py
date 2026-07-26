#!/usr/bin/env python3
"""Live monitor for the final dual-motor controller.

This program shows:
    - A 3D arm view
    - Target and measured joint angles
    - Target and measured angular velocity
    - Signed PWM commands

This final version does not create CSV files. It only reads live serial data,
updates the graphs, and sends keyboard commands to the controller.

Keyboard controls:
    j       Select Motor 1
    k       Select Motor 2
    0-9     Send a preset target angle
    x       Start or stop oscillation
    left/a  Move the selected motor in reverse/down
    right/d Move the selected motor forward/up
    s/e/p   Stop both motors
    space   Stop both motors
    r       Zero both IMUs and encoders
    m       Print the controller menu
    c       Clear values shown by Python
    g       Clear graph history
    q       Close the monitor
"""

# Name guide:
# ang = angle, vel = velocity, tgt = target, pwm = motor command
# prev = previous, hist = history, ser = serial, disp = display
# All custom variable names are 10 characters or less.

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
SER_PORT = "COM8"
BAUD_RATE = 230400
SER_TOUT = 0.1

# Reduced layout: the 3D arm is slightly smaller and the graphs get more width.
WIN_SIZE = (15, 10.5)
ARM_W = 0.72
GRAPH_W = 1.28

HIST_N = 1200
ANIM_MS = 35

# Approximate segment lengths used only by the 3D picture.
SH_TO_IMU = 1.0
IMU_TO_ELB = 5.25
ELB_TO_IMU = 3.0
IMU_TO_HND = 1.0

IMU_AXIS = np.array([1.0, 0.0, 0.0])
UP_DIR = -1.0
FR_DIR = -1.0

RAW_PRINT = False

# Motor 2 keeps a separate color so it is easy to distinguish.
M1_COLOR = "tab:blue"
M2_COLOR = "tab:orange"

# Used only when an older controller does not send VELOCITY data.
MEAS_A = 0.25
TGT_A = 0.50
MIN_DT = 0.001
MAX_DT = 0.250
VEL_TOL = 0.005


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
    vecQuat = np.array(
        [0.0, vector[0], vector[1], vector[2]],
        dtype=float,
    )

    rotated = multiply_quaternions(
        multiply_quaternions(quaternion, vecQuat),
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
DISP_BASE = np.array(
    [-0.7071067811865476, 0.0, 0.7071067811865476, 0.0]
)
DISP_ADD = np.array(
    [-0.7071067811865476, 0.0, 0.0, 0.7071067811865476]
)
DISP_ROT = multiply_quaternions(
    DISP_ADD,
    DISP_BASE,
)


# =====================================================
# Serial data and live values
# =====================================================

FLOAT_RE = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

QUAT_RE = re.compile(
    rf"^\s*(qUpperZeroed|qForearmZeroed|qJointZeroed)\s*:\s*"
    rf"({FLOAT_RE}),\s*({FLOAT_RE}),\s*"
    rf"({FLOAT_RE}),\s*({FLOAT_RE})\s*$"
)

serMsgs: deque[str] = deque()
stopProg = threading.Event()
ctrlSer: serial.Serial | None = None


def make_default_system_data() -> dict[str, object]:
    return {
        "upper_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "forearm_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "joint_quaternion": np.array([1.0, 0.0, 0.0, 0.0]),
        "board_time_s": 0.0,
        "selMotor": "M2",
        "m1_mode": "Idle",
        "m1_target": 0.0,
        "m1_current": 0.0,
        "m1_error": 0.0,
        "m1_pwm": 0,
        "m1_u_cmd": 0.0,
        "m1_counts": 0,
        "m1Vel": 0.0,
        "m1TgtVel": 0.0,
        "m2_mode": "Idle",
        "m2_target": 0.0,
        "m2_current": 0.0,
        "m2_error": 0.0,
        "m2_pwm": 0,
        "m2_u_cmd": 0.0,
        "m2_counts": 0,
        "m2Vel": 0.0,
        "m2TgtVel": 0.0,
        "m2_p_term": 0.0,
        "m2_i_term": 0.0,
        "m2_velocity_term": 0.0,
        "m2_raw_velocity_term": 0.0,
        "m2_velocity_limit_active": False,
        "m2_direction_protection_active": False,
        "m2_saved_drive_direction": 0,
        "m2_integral_state": 0.0,
        "m2_unsaturated_output": 0.0,
        "m2_saturated_output": 0.0,
        "m2_saturation_state": 0,
        "m2_active_pwm_limit": 0,
        "m2_slow_zone_active": False,
        "m2_brake_active": False,
        "m2_settled": False,
        "m2_actual_dt_s": 0.0,
        "m2_integration_dt_s": 0.0,
        "m2_long_control_gap": False,
        "control2_source": "Waiting for controller data",
        "velocity_source": "Waiting for controller data",
        "upAng": 0.0,
        "elbAng": 0.0,
        "spikeCnt": 0,
        "lines_read": 0,
        "quaternion_lines_read": 0,
    }


sysData = make_default_system_data()

# Previous values are kept only for the older-controller backup calculation.
_prevTime: float | None = None
_prevM1A: float = 0.0
_prevM1T: float = 0.0
_prevM2A: float = 0.0
_prevM2T: float = 0.0
_lastVelT: float | None = None

def clear_live_values() -> None:
    """Clear the values shown by Python without moving the motors."""
    sysData.clear()
    sysData.update(make_default_system_data())


def open_serial_port(port: str, baud_rate: int) -> serial.Serial | None:
    try:
        connection = serial.Serial(
            port,
            baud_rate,
            timeout=SER_TOUT,
        )
        print(f"Opened serial port {port} at {baud_rate} baud.")
        return connection

    except (serial.SerialException, OSError) as error:
        print(f"Could not open serial port {port}: {error}")
        return None


def send_serial_command(text: str) -> None:
    if ctrlSer is None or not ctrlSer.is_open:
        print("Serial port is not open.")
        return

    try:
        ctrlSer.write(text.encode("utf-8"))
        ctrlSer.flush()
        print(f"Sent: {text!r}")

    except (serial.SerialException, OSError) as error:
        print(f"Could not send command: {error}")


def read_serial_lines(connection: serial.Serial) -> None:
    try:
        while not stopProg.is_set():
            try:
                raw_line = connection.readline()
            except (serial.SerialException, OSError):
                break

            if not raw_line:
                continue

            line = raw_line.decode(errors="ignore").strip()

            if line:
                if RAW_PRINT:
                    print("RAW:", line)

                serMsgs.append(line)

    finally:
        try:
            connection.close()
        except (serial.SerialException, OSError):
            pass


def apply_low_pass_filter(previous: float, new_value: float, alpha: float) -> float:
    """Smooth a backup velocity value."""
    return alpha * new_value + (1.0 - alpha) * previous


def parse_velocity_line(line: str) -> bool:
    """Read exact velocity values used inside the controller.

    Expected format:
        VELOCITY,time_ms,
        m1_desired_velocity,m1_measured_velocity,
        m2_desired_velocity,m2_measured_velocity
    """
    global _lastVelT

    parts = [part.strip() for part in line.split(",")]

    if len(parts) != 6 or parts[0] != "VELOCITY":
        return False

    try:
        time_s = float(parts[1]) / 1000.0
        sysData["m1TgtVel"] = float(parts[2])
        sysData["m1Vel"] = float(parts[3])
        sysData["m2TgtVel"] = float(parts[4])
        sysData["m2Vel"] = float(parts[5])
    except ValueError:
        return False

    sysData["velocity_source"] = "Controller internal values"
    _lastVelT = time_s
    return True


def parse_control2_line(line: str) -> bool:
    """Read extra Motor 2 control values."""
    parts = [part.strip() for part in line.split(",")]

    # The current controller sends 20 fields. The older controller sent 16.
    # Accept both formats so this monitor can work with either version.
    if len(parts) not in {16, 20} or parts[0] != "CONTROL2":
        return False

    try:
        float(parts[1])  # Check that the timestamp is a number.
        sysData["m2_p_term"] = float(parts[2])
        sysData["m2_i_term"] = float(parts[3])
        sysData["m2_velocity_term"] = float(parts[4])
        sysData["m2_integral_state"] = float(parts[5])
        sysData["m2_unsaturated_output"] = float(parts[6])
        sysData["m2_saturated_output"] = float(parts[7])
        sysData["m2_saturation_state"] = int(parts[8])
        sysData["m2_active_pwm_limit"] = int(parts[9])
        sysData["m2_slow_zone_active"] = bool(int(parts[10]))
        sysData["m2_brake_active"] = bool(int(parts[11]))
        sysData["m2_settled"] = bool(int(parts[12]))
        sysData["m2_actual_dt_s"] = float(parts[13])
        sysData["m2_integration_dt_s"] = float(parts[14])
        sysData["m2_long_control_gap"] = bool(int(parts[15]))

        if len(parts) == 20:
            sysData["m2_raw_velocity_term"] = float(parts[16])
            sysData["m2_velocity_limit_active"] = bool(int(parts[17]))
            sysData["m2_direction_protection_active"] = bool(
                int(parts[18])
            )
            sysData["m2_saved_drive_direction"] = int(parts[19])
        else:
            sysData["m2_raw_velocity_term"] = sysData[
                "m2_velocity_term"
            ]
            sysData["m2_velocity_limit_active"] = False
            sysData["m2_direction_protection_active"] = False
            sysData["m2_saved_drive_direction"] = 0
    except ValueError:
        return False

    sysData["control2_source"] = "Controller internal values"
    return True

def parse_state_line(line: str) -> bool:
    """Read the compact STATE line sent by the dual-motor controller.

    The updated controller sends a matching VELOCITY line immediately before
    this line. When that message is missing, velocity is estimated from the
    change in angle divided by the change in time.
    """
    global _prevTime
    global _prevM1A
    global _prevM1T
    global _prevM2A
    global _prevM2T

    parts = [part.strip() for part in line.split(",")]

    if len(parts) != 20 or parts[0] != "STATE":
        return False

    try:
        time_s = float(parts[1]) / 1000.0
        selMotor = parts[2]

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

        upAng = float(parts[17])
        elbAng = float(parts[18])
        spikeCnt = int(parts[19])
    except ValueError:
        return False

    velFresh = (
        _lastVelT is not None
        and abs(_lastVelT - time_s)
        <= VEL_TOL
    )

    if not velFresh and _prevTime is not None:
        dt = time_s - _prevTime

        if MIN_DT <= dt <= MAX_DT:
            rawM1Vel = (m1_current - _prevM1A) / dt
            rawM1TgtV = (m1_target - _prevM1T) / dt
            rawM2Vel = (m2_current - _prevM2A) / dt
            rawM2TgtV = (m2_target - _prevM2T) / dt

            sysData["m1Vel"] = apply_low_pass_filter(
                float(sysData["m1Vel"]),
                rawM1Vel,
                MEAS_A,
            )
            sysData["m1TgtVel"] = apply_low_pass_filter(
                float(sysData["m1TgtVel"]),
                rawM1TgtV,
                TGT_A,
            )
            sysData["m2Vel"] = apply_low_pass_filter(
                float(sysData["m2Vel"]),
                rawM2Vel,
                MEAS_A,
            )
            sysData["m2TgtVel"] = apply_low_pass_filter(
                float(sysData["m2TgtVel"]),
                rawM2TgtV,
                TGT_A,
            )
            sysData["velocity_source"] = "Python backup calculation"

    sysData["board_time_s"] = time_s
    sysData["selMotor"] = selMotor

    sysData["m1_mode"] = m1_mode
    sysData["m1_target"] = m1_target
    sysData["m1_current"] = m1_current
    sysData["m1_error"] = m1_error
    sysData["m1_pwm"] = m1_pwm
    sysData["m1_u_cmd"] = m1_u_cmd
    sysData["m1_counts"] = m1_counts

    sysData["m2_mode"] = m2_mode
    sysData["m2_target"] = m2_target
    sysData["m2_current"] = m2_current
    sysData["m2_error"] = m2_error
    sysData["m2_pwm"] = m2_pwm
    sysData["m2_u_cmd"] = m2_u_cmd
    sysData["m2_counts"] = m2_counts

    sysData["upAng"] = upAng
    sysData["elbAng"] = elbAng
    sysData["spikeCnt"] = spikeCnt

    _prevTime = time_s
    _prevM1A = m1_current
    _prevM1T = m1_target
    _prevM2A = m2_current
    _prevM2T = m2_target

    return True


def parse_serial_line(line: str) -> None:
    sysData["lines_read"] = int(sysData["lines_read"]) + 1

    if line.startswith("VELOCITY,"):
        parse_velocity_line(line)
        return

    if line.startswith("CONTROL2,"):
        parse_control2_line(line)
        return

    if line.startswith("STATE,"):
        parse_state_line(line)
        return

    quatMatch = QUAT_RE.match(line)

    if quatMatch:
        label = quatMatch.group(1)
        quaternion = normalize_quaternion(
            np.array(
                [
                    float(quatMatch.group(2)),
                    float(quatMatch.group(3)),
                    float(quatMatch.group(4)),
                    float(quatMatch.group(5)),
                ]
            )
        )

        if label == "qUpperZeroed":
            sysData["upper_quaternion"] = quaternion
        elif label == "qForearmZeroed":
            sysData["forearm_quaternion"] = quaternion
        else:
            sysData["joint_quaternion"] = quaternion

        sysData["quaternion_lines_read"] = (
            int(sysData["quaternion_lines_read"]) + 1
        )
        return

    # Show controller messages such as selections, stops, and errors.
    print(line)


def process_serial_messages() -> None:
    while serMsgs:
        parse_serial_line(serMsgs.popleft())


# =====================================================
# 3D arm position calculation
# =====================================================

def calculate_arm_positions() -> tuple[np.ndarray, ...]:
    shoulder = np.array([0.0, 0.0, 0.0])
    base_axis = normalize_vector(IMU_AXIS)

    upDispQ = normalize_quaternion(
        multiply_quaternions(
            DISP_ROT,
            np.asarray(sysData["upper_quaternion"]),
        )
    )

    foreDispQ = normalize_quaternion(
        multiply_quaternions(
            DISP_ROT,
            np.asarray(sysData["forearm_quaternion"]),
        )
    )

    upDir = normalize_vector(
        rotate_vector_by_quaternion(
            base_axis * UP_DIR,
            upDispQ,
        )
    )

    foreDir = normalize_vector(
        rotate_vector_by_quaternion(
            base_axis * FR_DIR,
            foreDispQ,
        )
    )

    upper_imu = shoulder + SH_TO_IMU * upDir

    elbow = shoulder + (
        SH_TO_IMU + IMU_TO_ELB
    ) * upDir

    foreImu = elbow + ELB_TO_IMU * foreDir

    hand = elbow + (
        ELB_TO_IMU + IMU_TO_HND
    ) * foreDir

    return shoulder, upper_imu, elbow, foreImu, hand


# =====================================================
# Main visualizer
# =====================================================

def main() -> None:
    global ctrlSer

    ctrlSer = open_serial_port(SER_PORT, BAUD_RATE)

    if ctrlSer is None:
        return

    serThread = threading.Thread(
        target=read_serial_lines,
        args=(ctrlSer,),
        daemon=True,
    )
    serThread.start()

    # Three time plots remain. The narrower left column makes the 3D arm smaller.
    figure = plt.figure(figsize=WIN_SIZE)

    grid = figure.add_gridspec(
        3,
        2,
        width_ratios=[ARM_W, GRAPH_W],
        height_ratios=[1.0, 1.0, 1.0],
        wspace=0.32,
        hspace=0.62,
    )

    arm_axis = figure.add_subplot(grid[:, 0], projection="3d")
    angle_axis = figure.add_subplot(grid[0, 1])
    velAxis = figure.add_subplot(grid[1, 1])
    pwm_axis = figure.add_subplot(grid[2, 1])

    figure.subplots_adjust(
        left=0.04,
        right=0.97,
        top=0.96,
        bottom=0.11,
    )

    # Separate history is kept for both motors.
    timeHist: deque[float] = deque(maxlen=HIST_N)

    m1AngHist: deque[float] = deque(maxlen=HIST_N)
    m1TgtHist: deque[float] = deque(maxlen=HIST_N)
    m2AngHist: deque[float] = deque(maxlen=HIST_N)
    m2TgtHist: deque[float] = deque(maxlen=HIST_N)

    m1VHist: deque[float] = deque(maxlen=HIST_N)
    m1TVHist: deque[float] = deque(maxlen=HIST_N)
    m2VHist: deque[float] = deque(maxlen=HIST_N)
    m2TVHist: deque[float] = deque(maxlen=HIST_N)

    m1PwmHist: deque[float] = deque(maxlen=HIST_N)
    m2PwmHist: deque[float] = deque(maxlen=HIST_N)

    start_time = time.monotonic()

    # ---------------------
    # 3D arm panel
    # ---------------------

    armLen = (
        SH_TO_IMU
        + IMU_TO_ELB
        + ELB_TO_IMU
        + IMU_TO_HND
    )
    axis_limit = armLen + 0.10

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

    shoulder, upper_imu, elbow, foreImu, hand = (
        calculate_arm_positions()
    )

    shDot, = arm_axis.plot(
        [shoulder[0]],
        [shoulder[1]],
        [shoulder[2]],
        "o",
        markersize=7,
        label="Shoulder",
    )

    upImuDot, = arm_axis.plot(
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

    foreDot, = arm_axis.plot(
        [foreImu[0]],
        [foreImu[1]],
        [foreImu[2]],
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

    upArmLine, = arm_axis.plot(
        [shoulder[0], elbow[0]],
        [shoulder[1], elbow[1]],
        [shoulder[2], elbow[2]],
        linewidth=3,
        label="Motor 1 segment",
    )

    foreLine, = arm_axis.plot(
        [elbow[0], hand[0]],
        [elbow[1], hand[1]],
        [elbow[2], hand[2]],
        linewidth=3,
        label="Motor 2 segment",
    )

    # ---------------------
    # Graph 1: angles versus time
    # ---------------------

    m1AngLine, = angle_axis.plot(
        [],
        [],
        linewidth=2,
        color=M1_COLOR,
        label="M1 current",
    )
    m1TgtLine, = angle_axis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=M1_COLOR,
        label="M1 target",
    )
    m2AngLine, = angle_axis.plot(
        [],
        [],
        linewidth=2,
        color=M2_COLOR,
        label="M2 current",
    )
    m2TgtLine, = angle_axis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=M2_COLOR,
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

    m1VLine, = velAxis.plot(
        [],
        [],
        linewidth=2,
        color=M1_COLOR,
        label="M1 measured velocity",
    )
    m1TVLine, = velAxis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=M1_COLOR,
        label="M1 desired velocity",
    )
    m2VLine, = velAxis.plot(
        [],
        [],
        linewidth=2,
        color=M2_COLOR,
        label="M2 measured velocity",
    )
    m2TVLine, = velAxis.plot(
        [],
        [],
        linewidth=1.5,
        linestyle="--",
        color=M2_COLOR,
        label="M2 desired velocity",
    )

    velAxis.axhline(0.0, linewidth=1, alpha=0.5)
    velAxis.set_title(
        "Angular Velocity: Measured vs Desired",
        fontsize=10,
    )
    velAxis.set_xlabel("Time t (s)", fontsize=8)
    velAxis.set_ylabel("Velocity (degrees/s)", fontsize=8)
    velAxis.grid(True, alpha=0.3)
    velAxis.set_xlim(0.0, 1.0)
    velAxis.set_ylim(-30.0, 30.0)
    velAxis.legend(loc="upper right", fontsize=7, ncol=2)

    # ---------------------
    # Graph 3: signed PWM versus time
    # ---------------------

    m1PwmLine, = pwm_axis.plot(
        [],
        [],
        linewidth=2,
        color=M1_COLOR,
        label="M1 u(t)",
    )
    m2PwmLine, = pwm_axis.plot(
        [],
        [],
        linewidth=2,
        color=M2_COLOR,
        label="M2 u(t)",
    )

    pwm_axis.set_title(
        "Input u(t): Signed PWM Command vs Time",
        fontsize=10,
    )
    pwm_axis.set_xlabel("Time t (s)", fontsize=8)
    pwm_axis.set_ylabel("Signed PWM command", fontsize=8)
    pwm_axis.grid(True, alpha=0.3)
    pwm_axis.set_xlim(0.0, 1.0)
    pwm_axis.set_ylim(-260, 260)
    pwm_axis.legend(loc="upper right", fontsize=7)

    # Small status area at the bottom of the window.
    statText = figure.text(
        0.02,
        0.005,
        "",
        transform=figure.transFigure,
        verticalalignment="bottom",
        fontsize=7,
    )

    def reset_plot_history() -> None:
        nonlocal start_time

        timeHist.clear()

        m1AngHist.clear()
        m1TgtHist.clear()
        m2AngHist.clear()
        m2TgtHist.clear()

        m1VHist.clear()
        m1TVHist.clear()
        m2VHist.clear()
        m2TVHist.clear()

        m1PwmHist.clear()
        m2PwmHist.clear()

        start_time = time.monotonic()

        for line in (
            m1AngLine,
            m1TgtLine,
            m2AngLine,
            m2TgtLine,
            m1VLine,
            m1TVLine,
            m2VLine,
            m2TVLine,
            m1PwmLine,
            m2PwmLine,
        ):
            line.set_data([], [])

        angle_axis.set_xlim(0.0, 1.0)
        velAxis.set_xlim(0.0, 1.0)
        velAxis.set_ylim(-30.0, 30.0)
        pwm_axis.set_xlim(0.0, 1.0)

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
            # A one-letter command is more reliable than sending an ANSI
            # arrow-key code through the serial port.
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
            clear_live_values()
            print("Live values cleared.")

        elif key == "g":
            reset_plot_history()
            print("Graph history reset.")

        elif key == "q":
            stopProg.set()
            plt.close(figure)

    figure.canvas.mpl_connect("key_press_event", handle_key_press)

    def update_plot(_frame):
        process_serial_messages()

        # Update the 3D arm using the newest IMU quaternions.
        shoulder, upper_imu, elbow, foreImu, hand = (
            calculate_arm_positions()
        )

        points = [shoulder, upper_imu, elbow, foreImu, hand]
        dots = [
            shDot,
            upImuDot,
            elbow_dot,
            foreDot,
            hand_dot,
        ]

        for dot, point in zip(dots, points):
            dot.set_data([point[0]], [point[1]])
            dot.set_3d_properties([point[2]])

        upArmLine.set_data(
            [shoulder[0], elbow[0]],
            [shoulder[1], elbow[1]],
        )
        upArmLine.set_3d_properties(
            [shoulder[2], elbow[2]]
        )

        foreLine.set_data(
            [elbow[0], hand[0]],
            [elbow[1], hand[1]],
        )
        foreLine.set_3d_properties(
            [elbow[2], hand[2]]
        )

        runTime = time.monotonic() - start_time

        m1_current = float(sysData["m1_current"])
        m1_target = float(sysData["m1_target"])
        m1Pwm = float(sysData["m1_u_cmd"])
        m1Vel = float(sysData["m1Vel"])
        m1TgtVel = float(sysData["m1TgtVel"])

        m2_current = float(sysData["m2_current"])
        m2_target = float(sysData["m2_target"])
        m2Pwm = float(sysData["m2_u_cmd"])
        m2Vel = float(sysData["m2Vel"])
        m2TgtVel = float(sysData["m2TgtVel"])

        timeHist.append(runTime)

        m1AngHist.append(m1_current)
        m1TgtHist.append(m1_target)
        m2AngHist.append(m2_current)
        m2TgtHist.append(m2_target)

        m1VHist.append(m1Vel)
        m1TVHist.append(m1TgtVel)
        m2VHist.append(m2Vel)
        m2TVHist.append(m2TgtVel)

        m1PwmHist.append(m1Pwm)
        m2PwmHist.append(m2Pwm)

        # Graph 1: current and target angles.
        m1AngLine.set_data(timeHist, m1AngHist)
        m1TgtLine.set_data(timeHist, m1TgtHist)
        m2AngLine.set_data(timeHist, m2AngHist)
        m2TgtLine.set_data(timeHist, m2TgtHist)

        # Graph 2: target and measured velocity.
        m1VLine.set_data(timeHist, m1VHist)
        m1TVLine.set_data(
            timeHist,
            m1TVHist,
        )
        m2VLine.set_data(timeHist, m2VHist)
        m2TVLine.set_data(
            timeHist,
            m2TVHist,
        )

        # Graph 3: signed PWM command.
        m1PwmLine.set_data(timeHist, m1PwmHist)
        m2PwmLine.set_data(timeHist, m2PwmHist)

        if timeHist:
            left_time = timeHist[0]
            right_time = max(left_time + 1.0, timeHist[-1] + 0.5)

            angle_axis.set_xlim(left_time, right_time)
            velAxis.set_xlim(left_time, right_time)
            pwm_axis.set_xlim(left_time, right_time)

        velVals = (
            list(m1VHist)
            + list(m1TVHist)
            + list(m2VHist)
            + list(m2TVHist)
        )
        if velVals:
            velLow = min(velVals)
            velHigh = max(velVals)
            velSpan = max(velHigh - velLow, 20.0)
            velCtr = 0.5 * (velLow + velHigh)
            velMarg = 0.12 * velSpan
            velAxis.set_ylim(
                velCtr - 0.5 * velSpan - velMarg,
                velCtr + 0.5 * velSpan + velMarg,
            )

        statLines = [
            (
                "Keys: j select M1 | k select M2 | left/a right/d manual | "
                "0-9 target | x oscillation | s/space stop | r zero | "
                "g reset | q quit"
            ),
            (
                f"Selected: {sysData['selMotor']} | "
                f"M1 {sysData['m1_mode']}: target "
                f"{float(sysData['m1_target']):.1f}°, current "
                f"{float(sysData['m1_current']):.1f}°, error "
                f"{float(sysData['m1_error']):.1f}°, velocity "
                f"{float(sysData['m1Vel']):.2f}°/s, desired "
                f"{float(sysData['m1TgtVel']):.2f}°/s, PWM "
                f"{float(sysData['m1_u_cmd']):.0f}, counts "
                f"{sysData['m1_counts']}"
            ),
            (
                f"M2 {sysData['m2_mode']}: target "
                f"{float(sysData['m2_target']):.1f}°, current "
                f"{float(sysData['m2_current']):.1f}°, error "
                f"{float(sysData['m2_error']):.1f}°, velocity "
                f"{float(sysData['m2Vel']):.2f}°/s, desired "
                f"{float(sysData['m2TgtVel']):.2f}°/s, PWM "
                f"{float(sysData['m2_u_cmd']):.0f}, counts "
                f"{sysData['m2_counts']} | Velocity source: "
                f"{sysData['velocity_source']} | Rejected spikes: "
                f"{sysData['spikeCnt']}"
            ),
           
        ]

        statText.set_text("\n".join(statLines))

        return (
            *dots,
            upArmLine,
            foreLine,
            m1AngLine,
            m1TgtLine,
            m2AngLine,
            m2TgtLine,
            m1VLine,
            m1TVLine,
            m2VLine,
            m2TVLine,
            m1PwmLine,
            m2PwmLine,
            statText,
        )

    visAnim = animation.FuncAnimation(
        figure,
        update_plot,
        interval=ANIM_MS,
        blit=False,
        cache_frame_data=False,
    )

    # Keep a reference so Python does not delete the animation object.
    figure._visualizer_animation = visAnim

    try:
        plt.show()
    finally:
        stopProg.set()


if __name__ == "__main__":
    main()