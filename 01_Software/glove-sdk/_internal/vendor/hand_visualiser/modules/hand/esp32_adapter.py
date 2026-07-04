"""
ESP32 + 11x WT901 IMU adapter for glove hand pipeline.

Reads quaternion data from ESP32 serial output, maps 11 IMUs to 16 hand
joints, applies DIP coupling for the 5 missing distal joints, and corrects
the hand-back IMU (0x50) installation orientation.

IMU address -> hand joint mapping:
    0x50 -> Joint 0  (Wrist)          [180deg Y correction]
    0x51 -> Joint 13 (Thumb MCP)      0x52 -> Joint 14 (Thumb PIP)
    0x53 -> Joint 1  (Index MCP)      0x54 -> Joint 2  (Index PIP)
    0x55 -> Joint 4  (Middle MCP)     0x56 -> Joint 5  (Middle PIP)
    0x57 -> Joint 10 (Little MCP)     0x58 -> Joint 11 (Little PIP)
    0x59 -> Joint 7  (Ring MCP)       0x60 -> Joint 8  (Ring PIP)

DIP coupling (no physical IMU):
    Joint 3  (Index DIP)  <- scaled from Joint 2  (Index PIP)
    Joint 6  (Middle DIP) <- scaled from Joint 5  (Middle PIP)
    Joint 9  (Ring DIP)   <- scaled from Joint 8  (Ring PIP)
    Joint 12 (Little DIP) <- scaled from Joint 11 (Little PIP)
    Joint 15 (Thumb DIP)  <- scaled from Joint 14 (Thumb PIP)
"""

import re
import threading
import time

import numpy as np
import serial
import torch
from loguru import logger
from scipy.spatial.transform import Rotation as R

from modules.datamodels import ImuMsg
from modules.ring_buffer import RingBuffer

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------

NUM_IMUS = 11
NUM_HAND_JOINTS = 16

# IMU I2C address -> hand joint index
IMU_ADDR_TO_JOINT = {
    0x50: 0,   # Wrist
    0x51: 13,  # Thumb MCP
    0x52: 14,  # Thumb PIP
    0x53: 1,   # Index MCP
    0x54: 2,   # Index PIP
    0x55: 4,   # Middle MCP
    0x56: 5,   # Middle PIP
    0x57: 10,  # Little MCP
    0x58: 11,  # Little PIP
    0x59: 7,   # Ring MCP
    0x60: 8,   # Ring PIP
}

# PIP joint -> DIP joint pairs (DIP is coupled from PIP)
PIP_TO_DIP = {
    2: 3,    # Index  PIP -> DIP
    5: 6,    # Middle PIP -> DIP
    8: 9,    # Ring   PIP -> DIP
    11: 12,  # Little PIP -> DIP
    14: 15,  # Thumb  PIP -> DIP
}

DIP_JOINTS = set(PIP_TO_DIP.values())  # {3, 6, 9, 12, 15}

# DIP coupling ratios (θ_DIP ≈ ratio × θ_PIP)
DIP_COUPLING_RATIO_FINGER = 2.0 / 3.0
DIP_COUPLING_RATIO_THUMB = 0.6

HAND_BACK_ADDR = 0x50

# MCP joint indices used for wrist yaw consensus
MCP_JOINTS = [1, 4, 7, 10]  # Index, Middle, Ring, Little MCP

# Wrist yaw correction flag (mutable, set from UI)
wrist_yaw_correction = False

# 180-deg rotation around Y in scipy (x,y,z,w) format
Q_Y180_SCIPY = np.array([0.0, 1.0, 0.0, 0.0])

# Coordinate frame presets for WT901 → FSGlove mapping.
# Each preset is a rotation applied to every IMU quaternion before calibration.
# The calibrator was designed for HI229 IMUs; WT901 may use a different frame.
COORD_PRESETS = [
    ("None",           R.identity()),
    ("Rx+90",          R.from_euler('x',  90, degrees=True)),
    ("Rx-90",          R.from_euler('x', -90, degrees=True)),
    ("Ry+90",          R.from_euler('y',  90, degrees=True)),
    ("Ry-90",          R.from_euler('y', -90, degrees=True)),
    ("Rz+90",          R.from_euler('z',  90, degrees=True)),
    ("Rz-90",          R.from_euler('z', -90, degrees=True)),
    ("Rx180",          R.from_euler('x', 180, degrees=True)),
    ("Ry180",          R.from_euler('y', 180, degrees=True)),
    ("Rz180",          R.from_euler('z', 180, degrees=True)),
    ("Rx+90 Rz+90",   R.from_euler('xz', [ 90,  90], degrees=True)),
    ("Rx+90 Rz-90",   R.from_euler('xz', [ 90, -90], degrees=True)),
    ("Rx-90 Rz+90",   R.from_euler('xz', [-90,  90], degrees=True)),
    ("Rx-90 Rz-90",   R.from_euler('xz', [-90, -90], degrees=True)),
]

# Active coordinate preset index (mutable, set from UI)
active_coord_preset_idx = 0

# Serial line pattern: [0xNN] ... Q: w x y z
_LINE_PATTERN = re.compile(
    r'\[0x([0-9A-Fa-f]{2})\]'
    r'.*Q:\s*'
    r'([+-]?\d+\.?\d*)\s+'
    r'([+-]?\d+\.?\d*)\s+'
    r'([+-]?\d+\.?\d*)\s+'
    r'([+-]?\d+\.?\d*)'
)

# Frame header pattern (used as frame boundary)
_HEADER_PATTERN = re.compile(r'={4,}\s*IMU Direct Reader')


def parse_serial_line(line: str):
    """Parse one serial line and return (i2c_addr, quat_xyzw) or None."""
    m = _LINE_PATTERN.search(line)
    if m is None:
        return None
    addr = int(m.group(1), 16)
    w, x, y, z = float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))
    q = np.array([x, y, z, w])  # scipy (x,y,z,w)
    norm = np.linalg.norm(q)
    if norm < 1e-6:
        return None
    q /= norm
    return addr, q


def correct_hand_back_quat(q_xyzw: np.ndarray) -> np.ndarray:
    """Apply 180-deg Y rotation to hand-back IMU quaternion (body frame)."""
    r = R.from_quat(q_xyzw) * R.from_quat(Q_Y180_SCIPY)
    return r.as_quat()


def _circular_median(angles: list[float]) -> float:
    """Compute median of angles (radians), handling ±pi wrapping."""
    ref = angles[0]
    unwrapped = []
    for a in angles:
        diff = (a - ref + np.pi) % (2 * np.pi) - np.pi
        unwrapped.append(ref + diff)
    return float(np.median(unwrapped))


def correct_wrist_yaw_from_fingers(quats: np.ndarray) -> np.ndarray:
    """
    Replace wrist (Joint 0) yaw with median yaw from finger MCP joints.

    Pitch/Roll come from the wrist IMU's accelerometer (immune to magnetic
    interference).  Yaw is derived from finger MCP IMUs whose magnetometers
    are far from the interference source (battery / PCB).
    """
    identity = np.array([0.0, 0.0, 0.0, 1.0])

    mcp_yaws = []
    for j in MCP_JOINTS:
        if np.allclose(quats[j], identity, atol=1e-4):
            continue
        yaw = R.from_quat(quats[j]).as_euler('ZYX')[0]
        mcp_yaws.append(yaw)

    if len(mcp_yaws) < 2:
        return quats

    wrist_euler = R.from_quat(quats[0]).as_euler('ZYX')  # [yaw, pitch, roll]
    median_yaw = _circular_median(mcp_yaws)
    corrected = np.array([median_yaw, wrist_euler[1], wrist_euler[2]])
    quats[0] = R.from_euler('ZYX', corrected).as_quat()
    return quats


def build_16joint_quats(imu_quats: dict[int, np.ndarray]) -> np.ndarray:
    """
    Build (16, 4) quaternion array from up to 11 parsed IMU readings.

    - Applies active coordinate frame preset (WT901 → calibrator convention).
    - Maps each IMU address to its hand joint slot.
    - Corrects 0x50 for 180-deg Y installation offset.
    - Copies PIP quaternion into DIP slots (global orientation approximation).
    - Ensures all quaternions have unit norm (identity for missing joints).

    Returns (16, 4) in scipy (x,y,z,w) format.
    """
    global active_coord_preset_idx
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    quats = np.tile(identity, (NUM_HAND_JOINTS, 1))

    coord_rot = COORD_PRESETS[active_coord_preset_idx][1]

    for addr, q in imu_quats.items():
        joint = IMU_ADDR_TO_JOINT.get(addr)
        if joint is None:
            continue
        q_transformed = (coord_rot * R.from_quat(q)).as_quat()
        if addr == HAND_BACK_ADDR:
            q_transformed = correct_hand_back_quat(q_transformed)
        quats[joint] = q_transformed

    for pip_j, dip_j in PIP_TO_DIP.items():
        quats[dip_j] = quats[pip_j].copy()

    if wrist_yaw_correction:
        quats = correct_wrist_yaw_from_fingers(quats)

    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    zero_mask = (norms < 1e-6).flatten()
    quats[zero_mask] = identity
    norms[norms < 1e-6] = 1.0
    quats /= norms

    return quats


def apply_dip_coupling(hand_rotvec: torch.Tensor) -> torch.Tensor:
    """
    Post-process hand rotvec (16, 3): scale DIP joint rotations from PIP.

    The calibrator produces near-zero DIP rotations because the DIP
    quaternion equals the PIP quaternion.  We replace them with a biomechanical
    approximation: DIP_angle = ratio * PIP_angle.
    """
    out = hand_rotvec.clone()
    for pip_j, dip_j in PIP_TO_DIP.items():
        ratio = DIP_COUPLING_RATIO_THUMB if pip_j == 14 else DIP_COUPLING_RATIO_FINGER
        out[dip_j] = hand_rotvec[pip_j] * ratio
    return out


# ---------------------------------------------------------------------------
#  Serial reader thread
# ---------------------------------------------------------------------------

def esp32_serial_receiver(
    kill_switch: threading.Event,
    port: str,
    baudrate: int,
    buffer: RingBuffer,
):
    """Background thread that reads ESP32 serial and pushes ImuMsg into buffer."""
    logger.info(f"ESP32 serial receiver starting on {port} @ {baudrate}")
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baudrate
        ser.timeout = 1
        ser.dtr = False
        ser.rts = False
        ser.open()
    except serial.SerialException as e:
        logger.error(f"Cannot open {port}: {e}")
        return

    seq = 0
    frame_quats: dict[int, np.ndarray] = {}
    known_addrs = set(IMU_ADDR_TO_JOINT.keys())
    debug_line_count = 0
    min_imus_for_frame = 3

    def _flush_frame():
        nonlocal seq
        if len(frame_quats) >= min_imus_for_frame:
            quats_16 = build_16joint_quats(frame_quats)
            rotation = R.from_quat(quats_16)
            buffer.push(ImuMsg(
                sys_ticks=int(time.time() * 1e6),
                imu_rotation=rotation,
                seq=seq,
            ))
            if seq == 0:
                logger.info(f"First IMU frame: {len(frame_quats)} IMUs parsed "
                            f"(addrs: {[hex(a) for a in sorted(frame_quats.keys())]})")
            seq += 1
        frame_quats.clear()

    try:
        while not kill_switch.is_set():
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue

            debug_line_count += 1
            if debug_line_count <= 5:
                logger.debug(f"Serial line [{debug_line_count}]: {line[:160]}")

            if _HEADER_PATTERN.search(line):
                _flush_frame()
                continue

            parsed = parse_serial_line(line)
            if parsed is None:
                if debug_line_count <= 10:
                    logger.debug(f"Line not matched: {line[:100]}")
                continue

            addr, q = parsed
            if addr not in known_addrs:
                continue

            if addr in frame_quats:
                _flush_frame()

            frame_quats[addr] = q

    except Exception as e:
        logger.exception(f"ESP32 serial receiver error: {e}")
    finally:
        ser.close()
        logger.info("ESP32 serial receiver stopped")
