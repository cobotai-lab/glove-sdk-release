"""Minimal utils subset for glove_sdk vendor (stripped from fsglove utils.py)."""
import numpy as np
from scipy.spatial.transform import Rotation as R

_constant_frame_to_world_rot = R.from_euler(seq='xyz', angles=[np.pi / 2, 0, -np.pi]).inv()


def frame_to_world(rotation: R) -> R:
    """Convert hand rotation to world frame, so that a hand at rest pose is displayed as:
    - middle finger points to +X
    - palm points to -Z
    """
    return _constant_frame_to_world_rot * rotation
