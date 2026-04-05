from __future__ import annotations

from pathlib import Path
from typing import Dict
import yaml


def load_yaml(path: str | Path) -> dict:
    path = Path(path)
    with path.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def load_tag_map(path: str | Path) -> Dict[int, str]:
    data = load_yaml(path)
    mapping = data.get('tag_to_robot_id', {})
    out: Dict[int, str] = {}
    for k, v in mapping.items():
        out[int(k)] = str(v).strip().lower()
    return out


def ensure_robot_ns(robot_id: str) -> str:
    robot_id = robot_id.strip().lower()
    return f'/swarmbot_{robot_id}'


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    import math
    return math.atan2(siny_cosp, cosy_cosp)


def covariance_from_variances(
    x_var: float,
    y_var: float,
    z_var: float,
    roll_var: float,
    pitch_var: float,
    yaw_var: float,
) -> list[float]:
    cov = [0.0] * 36
    cov[0] = x_var
    cov[7] = y_var
    cov[14] = z_var
    cov[21] = roll_var
    cov[28] = pitch_var
    cov[35] = yaw_var
    return cov
