"""
Pure-Python obstacle avoidance decision logic (no ROS imports).

Kept separate from the ROS node so it can be unit tested without ROS.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence

# Tolerance so beams exactly on a sector edge (e.g. +30 deg) are not lost to
# floating point error in angle_min + i * angle_increment.
ANGLE_EPS = 1e-6

FORWARD = 'forward'
TURN_LEFT = 'turn_left'
TURN_RIGHT = 'turn_right'
STOP = 'stop'


@dataclass(frozen=True)
class AvoidanceParams:
    """Tunable parameters of the obstacle avoider (SI units, radians)."""

    obstacle_threshold: float = 0.6
    forward_speed: float = 0.2
    turn_speed: float = 0.6
    front_half_angle: float = math.radians(30.0)
    side_max_angle: float = math.radians(90.0)

    def __post_init__(self) -> None:
        """Reject values that would make the behavior meaningless or unsafe."""
        if not self.obstacle_threshold > 0.0:
            raise ValueError('obstacle_threshold must be > 0')
        if self.forward_speed < 0.0 or self.turn_speed < 0.0:
            raise ValueError('forward_speed and turn_speed must be >= 0')
        if not 0.0 < self.front_half_angle < self.side_max_angle <= math.pi:
            raise ValueError('need 0 < front_half_angle < side_max_angle <= pi')


@dataclass(frozen=True)
class Decision:
    """Velocity command chosen for one scan."""

    linear: float
    angular: float
    state: str


STOP_DECISION = Decision(0.0, 0.0, STOP)


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the range [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clean_range(value: float, range_min: float, range_max: float) -> Optional[float]:
    """
    Return a usable distance for one beam, or None if the beam is invalid.

    +Inf means no return, so the beam is free up to range_max (REP 117).
    -Inf and finite readings below range_min mean something is closer than
    the sensor can measure; they are treated conservatively as an obstacle at
    range_min (e.g. a wall touching the bumper, which is ~0.075 m in front of
    the LiDAR). NaN is discarded. Readings above range_max are clamped.
    """
    if math.isnan(value):
        return None
    if value == -math.inf or value < range_min:
        return range_min
    return min(value, range_max)


def sector_values(ranges: Sequence[float], angle_min: float, angle_increment: float,
                  lo: float, hi: float, range_min: float,
                  range_max: float) -> List[float]:
    """Return cleaned ranges whose beam angle lies within [lo, hi] (radians)."""
    values: List[float] = []
    for i, raw in enumerate(ranges):
        angle = normalize_angle(angle_min + i * angle_increment)
        if lo - ANGLE_EPS <= angle <= hi + ANGLE_EPS:
            cleaned = clean_range(raw, range_min, range_max)
            if cleaned is not None:
                values.append(cleaned)
    return values


def decide(ranges: Sequence[float], angle_min: float, angle_increment: float,
           range_min: float, range_max: float, params: AvoidanceParams,
           previous_state: str = FORWARD) -> Decision:
    """
    Choose a velocity command from one laser scan.

    Drive forward while every valid beam in the front sector
    (+/- front_half_angle) is farther than obstacle_threshold. Otherwise turn
    in place toward the side (left or right sector, between front_half_angle
    and side_max_angle) with more average free space. While already turning,
    the previous direction is kept until the front is clear, which avoids
    left/right oscillation in front of a flat wall. If the front sector has
    no valid beam at all (e.g. only NaN), the robot stops.
    """
    half = params.front_half_angle
    front = sector_values(ranges, angle_min, angle_increment, -half, half,
                          range_min, range_max)
    if not front:
        return STOP_DECISION
    if min(front) > params.obstacle_threshold:
        return Decision(params.forward_speed, 0.0, FORWARD)

    if previous_state == TURN_LEFT:
        return Decision(0.0, params.turn_speed, TURN_LEFT)
    if previous_state == TURN_RIGHT:
        return Decision(0.0, -params.turn_speed, TURN_RIGHT)

    left = sector_values(ranges, angle_min, angle_increment, half,
                         params.side_max_angle, range_min, range_max)
    right = sector_values(ranges, angle_min, angle_increment,
                          -params.side_max_angle, -half, range_min, range_max)
    left_space = sum(left) / len(left) if left else 0.0
    right_space = sum(right) / len(right) if right else 0.0

    if left_space >= right_space:
        return Decision(0.0, params.turn_speed, TURN_LEFT)
    return Decision(0.0, -params.turn_speed, TURN_RIGHT)


def is_stale(last_scan_ns: Optional[int], now_ns: int, timeout_s: float) -> bool:
    """
    Return True if no scan was received yet or the last one is too old.

    A negative age (clock jumped backwards, e.g. a simulation reset) also
    counts as stale, so the robot stops until fresh data arrives.
    """
    if last_scan_ns is None:
        return True
    age_ns = now_ns - last_scan_ns
    return age_ns < 0 or age_ns > timeout_s * 1e9
