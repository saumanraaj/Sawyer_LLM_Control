"""
vision_engine/utils.py — Shared Utilities

Provides common functions used across all scripts:
  - Configuration loading
  - Calibration loading
  - Calibration tool filtering
"""

import os
import numpy as np
import yaml
from typing import Optional


def load_config(config_path: str) -> dict:
    """
    Load the scene configuration YAML file.

    Parameters
    ----------
    config_path : str
        Path to scene_config.yaml.

    Returns
    -------
    config : dict
        Parsed configuration.
    """
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def load_calibration(config: dict, base_dir: str = ".") -> Optional[np.ndarray]:
    """
    Load the calibration transform from config/calibration.yaml if it exists.

    Parameters
    ----------
    config : dict
        Parsed scene_config.yaml.
    base_dir : str
        Base directory for resolving relative paths (default: cwd).

    Returns
    -------
    transform : np.ndarray (4, 4) or None
        Calibration transform from OptiTrack frame to world frame.
        Returns None if calibration file does not exist.
    """
    cal_cfg = config.get("calibration", {})
    cal_file = cal_cfg.get("file", "config/calibration.yaml")

    # Resolve relative to base_dir
    if not os.path.isabs(cal_file):
        cal_file = os.path.join(base_dir, cal_file)

    if not os.path.exists(cal_file):
        return None

    with open(cal_file, "r") as f:
        cal_data = yaml.safe_load(f)

    if cal_data is None or "transform" not in cal_data:
        print(f"[Utils] Warning: {cal_file} exists but has no 'transform' key.")
        return None

    T = np.array(cal_data["transform"], dtype=np.float64)
    if T.shape != (4, 4):
        print(f"[Utils] Warning: Transform in {cal_file} has shape {T.shape}, expected (4,4).")
        return None

    method = cal_data.get("method", "unknown")
    print(f"[Utils] Loaded calibration from {cal_file} (method: {method})")
    return T


def get_calibration_tool_name(config: dict) -> Optional[str]:
    """
    Get the rigid body name of the calibration tool from config.

    Returns
    -------
    name : str or None
        Rigid body name (e.g., "CS-100"), or None if not configured.
    """
    return config.get("calibration", {}).get("tool_body", None)


def is_calibration_tool(name: str, config: dict) -> bool:
    """
    Check if a rigid body name is the calibration tool.

    Checks both the calibration.tool_body config and the
    objects.*.is_calibration_tool flag.
    """
    # Check calibration section
    tool_name = get_calibration_tool_name(config)
    if tool_name and name == tool_name:
        return True

    # Check objects section
    obj_info = config.get("objects", {}).get(name, {})
    return obj_info.get("is_calibration_tool", False)
