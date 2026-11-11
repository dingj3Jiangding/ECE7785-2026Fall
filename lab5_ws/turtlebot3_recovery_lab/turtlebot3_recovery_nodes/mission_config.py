"""Validate lab mission files without importing ROS (positions in metres, yaw in radians)."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Mission:
    frame_id: str
    waypoints: tuple
    recovery_waypoints: tuple


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{label} must be a finite number')
    return result


def _waypoints(raw, label):
    if not isinstance(raw, list):
        raise ValueError(f'{label} must be a list of {{x, y, yaw}} mappings')
    result = []
    names = set()
    for index, item in enumerate(raw, 1):
        where = f'{label}[{index}]'
        if not isinstance(item, dict):
            raise ValueError(f'{where} must be a mapping')
        extra = set(item) - {'name', 'x', 'y', 'yaw'}
        if extra:
            raise ValueError(f'{where}: unexpected keys {sorted(map(str, extra))}')
        if 'x' not in item or 'y' not in item:
            raise ValueError(f'{where} requires x and y')
        name = item.get('name', f'{label}_{index}')
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError(f'{where}.name must be a unique nonempty string')
        names.add(name)
        result.append(Waypoint(
            name, _number(item['x'], f'{where}.x'),
            _number(item['y'], f'{where}.y'),
            _number(item.get('yaw', 0.0), f'{where}.yaw')))
    return tuple(result)


def load_mission(path, require_waypoints=False, require_recovery=False):
    """Read one shared mission for the route client and recovery service.

    Empty lists deliberately remain valid for a configuration template; callers
    must require the list they are about to execute. No coordinates are invented.
    """
    if not isinstance(path, (str, Path)) or not str(path).strip():
        raise ValueError('mission_file must name a YAML file')
    try:
        with Path(path).expanduser().open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'Cannot read mission file {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError('Mission YAML must be a mapping')
    extra = set(data) - {'frame_id', 'waypoints', 'recovery_waypoints'}
    if extra:
        raise ValueError(f'Unexpected mission keys: {sorted(map(str, extra))}')
    frame_id = data.get('frame_id', 'map')
    if (not isinstance(frame_id, str) or not frame_id.strip()
            or frame_id != frame_id.strip() or frame_id.startswith('/')):
        raise ValueError('frame_id must be a nonempty TF frame without a leading slash')
    waypoints = _waypoints(data.get('waypoints', []), 'waypoints')
    recovery = _waypoints(data.get('recovery_waypoints', []), 'recovery_waypoints')
    if require_waypoints and not waypoints:
        raise ValueError('Fill waypoints with the goals measured in your map before starting')
    if require_recovery and not recovery:
        raise ValueError('Fill recovery_waypoints with at least one verified safe map location')
    return Mission(frame_id, waypoints, recovery)
