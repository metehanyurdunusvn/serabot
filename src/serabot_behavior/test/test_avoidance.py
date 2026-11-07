"""Unit tests for the pure obstacle avoidance logic (no ROS needed)."""

import math
from typing import List

from serabot_behavior.avoidance import (AvoidanceParams, clean_range, decide, Decision,
                                        FORWARD, is_stale, normalize_angle, sector_values,
                                        STOP, TURN_LEFT, TURN_RIGHT)

# Same layout as the SeraBot LiDAR: 360 beams, angle_min = -pi, 1 deg steps.
# Index 180 = front (0 deg), 270 = left (+90 deg), 90 = right (-90 deg).
N = 360
ANGLE_MIN = -math.pi
INC = 2.0 * math.pi / N
RANGE_MIN = 0.12
RANGE_MAX = 10.0
PARAMS = AvoidanceParams()


def scan(default: float = 5.0) -> List[float]:
    return [default] * N


def set_deg(ranges: List[float], lo_deg: int, hi_deg: int, value: float) -> None:
    """Set beams with angle in [lo_deg, hi_deg] (robot frame, degrees)."""
    for deg in range(lo_deg, hi_deg + 1):
        ranges[180 + deg] = value


def run(ranges: List[float], previous: str = FORWARD) -> Decision:
    return decide(ranges, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX, PARAMS, previous)


def test_clear_front_moves_forward() -> None:
    d = run(scan())
    assert d.state == FORWARD
    assert d.linear == PARAMS.forward_speed
    assert d.angular == 0.0


def test_obstacle_in_front_turns_toward_more_space_left() -> None:
    r = scan()
    set_deg(r, -10, 10, 0.4)
    set_deg(r, -90, -31, 0.8)  # right side cramped
    d = run(r)
    assert d.state == TURN_LEFT
    assert d.linear == 0.0
    assert d.angular > 0.0


def test_obstacle_in_front_turns_toward_more_space_right() -> None:
    r = scan()
    set_deg(r, -10, 10, 0.4)
    set_deg(r, 31, 90, 0.8)  # left side cramped
    d = run(r)
    assert d.state == TURN_RIGHT
    assert d.angular < 0.0


def test_threshold_is_strict_and_sector_edges_count() -> None:
    r = scan()
    r[180 + 30] = PARAMS.obstacle_threshold  # exactly at threshold, on the +30 deg edge
    assert run(r).state != FORWARD
    r = scan()
    r[180 - 30] = 0.3  # on the -30 deg edge
    assert run(r).state != FORWARD
    r = scan()
    r[180 + 35] = 0.2  # outside the front sector: ignored
    assert run(r).state == FORWARD


def test_rear_blind_spot_is_ignored() -> None:
    r = scan()
    r[0] = 0.28  # mast seen behind the robot
    r[359] = 0.28
    assert run(r).state == FORWARD


def test_keeps_turn_direction_until_front_clear() -> None:
    r = scan()
    set_deg(r, -10, 10, 0.4)
    set_deg(r, 31, 90, 0.8)  # would choose right from scratch
    assert run(r, previous=TURN_LEFT).state == TURN_LEFT
    set_deg(r, -10, 10, 5.0)
    assert run(r, previous=TURN_LEFT).state == FORWARD


def test_inf_means_free_and_nan_is_ignored() -> None:
    r = scan(math.inf)
    r[180] = math.nan
    assert run(r).state == FORWARD


def test_too_close_readings_count_as_obstacle() -> None:
    r = scan()
    r[180] = 0.05  # below range_min: e.g. wall touching the bumper
    assert run(r).state != FORWARD
    r = scan()
    r[180] = -math.inf
    assert run(r).state != FORWARD


def test_no_valid_front_beam_stops() -> None:
    r = scan()
    set_deg(r, -30, 30, math.nan)
    d = run(r)
    assert d.state == STOP
    assert d.linear == 0.0 and d.angular == 0.0
    assert decide([], ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX, PARAMS).state == STOP


def test_scan_starting_at_zero_angle() -> None:
    # 0..2pi layout: index 0 = front; wrap-around must still form the front sector.
    r = [5.0] * N
    r[350] = 0.3  # -10 deg
    d = decide(r, 0.0, INC, RANGE_MIN, RANGE_MAX, PARAMS)
    assert d.state != FORWARD


def test_clean_range() -> None:
    assert clean_range(math.nan, RANGE_MIN, RANGE_MAX) is None
    assert clean_range(math.inf, RANGE_MIN, RANGE_MAX) == RANGE_MAX
    assert clean_range(-math.inf, RANGE_MIN, RANGE_MAX) == RANGE_MIN
    assert clean_range(0.01, RANGE_MIN, RANGE_MAX) == RANGE_MIN
    assert clean_range(42.0, RANGE_MIN, RANGE_MAX) == RANGE_MAX
    assert clean_range(1.5, RANGE_MIN, RANGE_MAX) == 1.5


def test_sector_values_and_normalize() -> None:
    assert math.isclose(normalize_angle(3.0 * math.pi / 2.0), -math.pi / 2.0)
    r = scan()
    vals = sector_values(r, ANGLE_MIN, INC, -math.radians(30.5), math.radians(30.5),
                         RANGE_MIN, RANGE_MAX)
    assert len(vals) == 61  # -30..+30 deg inclusive


def test_is_stale() -> None:
    assert is_stale(None, 0, 0.5)
    assert not is_stale(1_000_000_000, 1_400_000_000, 0.5)
    assert is_stale(1_000_000_000, 1_600_000_000, 0.5)
    assert is_stale(2_000_000_000, 1_000_000_000, 0.5)  # clock jumped back


def test_invalid_params_rejected() -> None:
    for kwargs in ({'obstacle_threshold': 0.0},
                   {'forward_speed': -0.1},
                   {'front_half_angle': math.radians(100.0)}):
        try:
            AvoidanceParams(**kwargs)
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {kwargs}')
