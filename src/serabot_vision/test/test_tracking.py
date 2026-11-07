"""Offline red detection and LiDAR controller regression tests."""
import math

import cv2
import numpy as np

from serabot_behavior.avoidance import AvoidanceParams, FORWARD
from serabot_vision.red_detection import RedTarget, find_red_target
from serabot_vision.tracking import (
    ALIGN, APPROACH, AVOID, AVOID_TURN, AVOID_DRIVE, TARGET_STOP,
    TrackingParams, VisionNavigator, choose_velocity, corridor_distance,
)


def scan(front=3.0):
    ranges = [4.0] * 360
    for j in range(-14, 15):
        ranges[(j + 180) % 360] = front
    return ranges


def act(ranges, target=None, reached=False):
    return choose_velocity(ranges, -math.pi, math.pi / 180, 0.12, 10.0, target,
                           AvoidanceParams(), TrackingParams(),
                           FORWARD, reached)


def navstep(nav, ranges, target, pose):
    return nav.step(ranges, -math.pi, math.pi / 180, 0.12, 10.0, target, pose)


def test_detect_red_target():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (460, 240), 35, (0, 0, 255), -1)
    target = find_red_target(image)
    assert target is not None and target.offset > 0.3
    assert target.area > 3000 and target.width >= 65


def test_brown_pot_is_not_red():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (320, 240), 35, (31, 69, 140), -1)
    assert find_red_target(image) is None


def test_ignore_small_noise():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (320, 240), 3, (0, 0, 255), -1)
    assert find_red_target(image) is None


def test_rotate_toward_right_target():
    decision = act(scan(), RedTarget(0.6, 500, 490, 240, 50))
    assert decision.state == ALIGN and decision.linear == 0 and decision.angular < 0


def test_approach_when_centered_and_clear():
    decision = act(scan(2.0), RedTarget(0.0, 500, 320, 240, 50))
    assert decision.state == APPROACH and 0 < decision.linear <= 0.12


def test_stop_at_safe_distance():
    decision = act(scan(0.6), RedTarget(0.0, 500, 320, 240, 160))
    assert decision.state == TARGET_STOP and decision.linear == 0


def test_near_green_pot_not_red_goal():
    decision = act(scan(0.6), RedTarget(0.0, 500, 320, 240, 35))
    assert decision.state == AVOID and decision.linear == 0


def test_latched_stop():
    decision = act(scan(2.0), None, True)
    assert decision.state == TARGET_STOP and decision.linear == 0


def test_no_lidar_beams_no_motion():
    decision = act([float('nan')] * 360, RedTarget(0.0, 500, 320, 240, 35))
    assert decision.linear == 0


def test_guard_blocked_wide_sector():
    ranges = scan(2.0)
    ranges[180 + 20] = 0.30
    decision = act(ranges, RedTarget(0.0, 500, 320, 240, 35))
    assert decision.state == AVOID and decision.linear == 0


def test_patrol_when_no_target():
    decision = act(scan(2.0), None)
    assert decision.linear > 0


def test_detour_turn_then_drive_then_resume():
    nav = VisionNavigator()
    blocked = scan(0.65)
    target = RedTarget(0.0, 500, 320, 240, 30)
    first = navstep(nav, blocked, target, (0.0, 0.0, 0.0))
    assert first.state == AVOID_TURN and first.linear == 0.0
    turn_yaw = nav.direction * 0.5
    second = navstep(nav, scan(3.0), target, (0.0, 0.0, turn_yaw))
    assert second.state == AVOID_DRIVE and second.linear > 0
    third = navstep(nav, scan(3.0), target, (0.5, 0.0, turn_yaw))
    assert third.state == AVOID_DRIVE and third.linear > 0
    fourth = navstep(nav, scan(3.0), target, (1.0, 0.0, turn_yaw))
    assert fourth.state in (APPROACH, ALIGN)


def test_detour_replans_if_forward_blocked():
    nav = VisionNavigator()
    target = RedTarget(0.0, 500, 320, 240, 30)
    navstep(nav, scan(0.65), target, (0, 0, 0))
    drive = navstep(nav, scan(3.0), target, (0, 0, nav.direction*0.5))
    assert drive.state == AVOID_DRIVE
    next_step = navstep(nav, scan(0.45), target, (0.1, 0, nav.direction*0.5))
    assert next_step.state == AVOID_TURN and next_step.linear == 0


def test_fail_safe_on_missing_odometry():
    nav = VisionNavigator()
    result = navstep(nav, scan(), RedTarget(0, 500, 320, 240, 30), None)
    assert result.linear == 0 and result.angular == 0


def test_remembered_goal_rotates_back_after_detour():
    nav = VisionNavigator()
    red = RedTarget(0.0, 3200, 320, 240, 70)
    navstep(nav, scan(3.0), red, (0.0, 0.0, 0.0))
    assert nav.goal_xy is not None and nav.goal_xy[0] > 2
    # Robot turned right and drove while the red target left the camera view.
    step = navstep(nav, scan(3.0), None, (0.8, -0.3, -0.35))
    assert step.state == ALIGN and step.linear == 0 and step.angular > 0


def test_goal_not_updated_with_clipped_edge_blob():
    nav = VisionNavigator()
    good = RedTarget(0.0, 3200, 320, 240, 70)
    navstep(nav, scan(3.0), good, (0.0, 0.0, 0.0))
    old_goal = nav.goal_xy
    clipped = RedTarget(-0.99, 160, 5, 240, 11)
    navstep(nav, scan(3.0), clipped, (0.0, 0.0, 0.0))
    assert nav.goal_xy == old_goal


def test_near_red_alignment_precedes_detour():
    ranges = scan(0.70)
    target = RedTarget(-0.35, 6000, 160, 240, 180)
    decision = act(ranges, target)
    assert decision.state == ALIGN and decision.linear == 0 and decision.angular > 0


def test_vision_confirmed_standoff_stop():
    target = RedTarget(0.0, 12000, 320, 240, 190)
    decision = act(scan(0.90), target)
    assert decision.state == TARGET_STOP and decision.linear == 0


def test_lateral_pot_not_in_driving_corridor():
    ranges = scan(2.0)
    ranges[180 + 30] = 0.65
    close = corridor_distance(ranges, -math.pi, math.pi / 180, 0.12, 10.0)
    assert close > 1.0
    target = RedTarget(0.0, 500, 320, 240, 40)
    assert act(ranges, target).state == APPROACH


def test_pot_in_driving_corridor_triggers_avoidance():
    ranges = scan(2.0)
    ranges[180 + 20] = 0.55
    target = RedTarget(0.0, 500, 320, 240, 40)
    assert act(ranges, target).state == AVOID
