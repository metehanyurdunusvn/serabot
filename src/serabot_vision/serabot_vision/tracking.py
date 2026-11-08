"""Pure vision + LiDAR navigation logic and stateful reactive detour controller."""
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from serabot_behavior.avoidance import (
    AvoidanceParams, Decision, FORWARD, STOP_DECISION, decide,
    normalize_angle, sector_values,
)
from .red_detection import RedTarget

SEARCH = 'search'
ALIGN = 'align_target'
APPROACH = 'approach_target'
AVOID = 'avoid_obstacle'
AVOID_TURN = 'avoid_turn'
AVOID_DRIVE = 'avoid_drive'
TARGET_STOP = 'target_reached'

Pose = Tuple[float, float, float]


@dataclass(frozen=True)
class TrackingParams:
    stop_distance: float = 0.65
    approach_speed: float = 0.20
    turn_speed: float = 0.60
    center_tolerance: float = 0.12
    camera_hfov: float = 1.047
    front_half_angle: float = math.radians(14)
    # Start a detour BEFORE the robot has drifted into an adjacent plant row.
    obstacle_clearance: float = 0.80
    # Red sphere is 0.36m wide; this width rules out a distant red blob behind
    # a closer *green* plant's pot being mistaken for the reached target.
    min_stop_target_width: int = 115
    detour_turn_angle: float = 0.40
    detour_forward_distance: float = 0.85
    detour_speed: float = 0.22


def front_distance(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, half_angle: float) -> Optional[float]:
    values = sector_values(ranges, angle_min, angle_increment, -half_angle, half_angle,
                           range_min, range_max)
    return min(values) if values else None


def corridor_distance(ranges: Sequence[float], angle_min: float, angle_increment: float,
                      range_min: float, range_max: float,
                      half_width: float = 0.30) -> Optional[float]:
    """Nearest obstacle along a body-width forward corridor.

    A pot at +30 degrees may be close, but is not on the robot's collision
    path. Reject those false positives while still guarding the chassis.
    """
    points = []
    for i, raw in enumerate(ranges):
        angle = normalize_angle(angle_min + i * angle_increment)
        if abs(angle) > math.radians(55):
            continue
        if math.isnan(raw):
            continue
        if raw == -math.inf or raw < range_min:
            raw = range_min
        elif raw == math.inf or raw > range_max:
            raw = range_max
        x = raw * math.cos(angle)
        y = raw * math.sin(angle)
        if x >= 0 and abs(y) <= half_width:
            points.append(x)
    return min(points) if points else None


def free_side(ranges: Sequence[float], angle_min: float, angle_increment: float,
              range_min: float, range_max: float) -> int:
    """Choose left (+1) or right (-1) using available side LiDAR range."""
    left = sector_values(ranges, angle_min, angle_increment, math.radians(20),
                         math.radians(95), range_min, range_max)
    right = sector_values(ranges, angle_min, angle_increment, math.radians(-95),
                          math.radians(-20), range_min, range_max)
    # Lower percentile discourages turning toward a side with close plants.
    def score(values):
        if not values:
            return -1.0
        v = sorted(values)
        return v[max(0, len(v) // 4)]
    return 1 if score(left) >= score(right) else -1


def choose_velocity(ranges: Sequence[float], angle_min: float, angle_increment: float,
                    range_min: float, range_max: float, target: Optional[RedTarget],
                    avoidance: AvoidanceParams, tracking: TrackingParams,
                    previous_motion: str = FORWARD,
                    target_reached: bool = False) -> Decision:
    """Evaluate immediate LiDAR-safe behavior; AVOID triggers a stateful detour."""
    if target_reached:
        return Decision(0.0, 0.0, TARGET_STOP)
    front = front_distance(ranges, angle_min, angle_increment, range_min, range_max,
                           tracking.front_half_angle)
    if front is None:
        return STOP_DECISION
    if target is None:
        return decide(ranges, angle_min, angle_increment, range_min, range_max,
                      avoidance, previous_motion)

    # A nearby red sphere is more reliable than a distant clipped blob.
    # Prioritize rotating in place to center it (never forward while turning).
    close_red = target.width >= tracking.min_stop_target_width
    aligned = abs(target.offset) <= tracking.center_tolerance
    if close_red and not aligned:
        return Decision(0.0, -math.copysign(tracking.turn_speed, target.offset), ALIGN)

    if aligned and close_red and front <= tracking.stop_distance:
        return Decision(0.0, 0.0, TARGET_STOP)

    # The robot can also stop at a conservative standoff if the visually
    # confirmed target is large and LiDAR reports an object less than 0.95m
    # straight ahead. This avoids another detour when already by the plant.
    if aligned and target.width >= 165 and front <= 0.95:
        return Decision(0.0, 0.0, TARGET_STOP)

    corridor = corridor_distance(ranges, angle_min, angle_increment,
                                 range_min, range_max)
    if corridor is None:
        return STOP_DECISION
    if corridor <= tracking.obstacle_clearance:
        return Decision(0.0, 0.0, AVOID)

    if not aligned:
        return Decision(0.0, -math.copysign(tracking.turn_speed, target.offset), ALIGN)

    speed = min(tracking.approach_speed,
                max(0.035, (front - tracking.stop_distance) * 0.28))
    turn = max(-tracking.turn_speed * 0.5,
               min(tracking.turn_speed * 0.5,
                   -target.offset * tracking.turn_speed))
    return Decision(speed, turn, APPROACH)


class VisionNavigator:
    """Stateful obstacle detours, then return to camera servoing.

    ROS-free: pose is the current odometry (x,y,yaw) and sensor arrays are
    explicit. While detouring, the robot turns away from the obstacle, drives
    past it for an odometry-measured distance, then tries reacquiring red.
    """

    def __init__(self, avoidance: Optional[AvoidanceParams] = None,
                 tracking: Optional[TrackingParams] = None) -> None:
        self.avoidance = avoidance or AvoidanceParams()
        self.tracking = tracking or TrackingParams()
        self.mode = SEARCH
        self.direction = 1
        self.start_pose: Optional[Pose] = None
        self.reached = False
        self.previous_motion = FORWARD
        self.goal_xy: Optional[Tuple[float, float]] = None

    def remember_target(self, target: Optional[RedTarget], pose: Pose) -> None:
        """Estimate a world-fixed target direction from the camera and odometry.

        The Gazebo red sphere is 0.36m wide. Its observed angular diameter
        yields approximate distance; we use this only for reorientation after
        a detour, never as the independent stop condition.
        """
        if target is None or target.width < 24 or abs(target.offset) > 0.78:
            return
        half_fov = self.tracking.camera_hfov / 2
        focal_px = 320.0 / math.tan(half_fov)
        distance = min(10.0, 0.36 * focal_px / target.width)
        bearing = -math.atan(target.offset * math.tan(half_fov))
        angle = pose[2] + bearing
        measured = (pose[0] + distance * math.cos(angle),
                    pose[1] + distance * math.sin(angle))
        if self.goal_xy is None:
            self.goal_xy = measured
        else:
            # Larger/nearer blobs tend to be more reliable measurements.
            alpha = 0.35 if target.width >= 65 else 0.12
            self.goal_xy = (self.goal_xy[0] * (1 - alpha) + measured[0] * alpha,
                            self.goal_xy[1] * (1 - alpha) + measured[1] * alpha)

    def remembered_target(self, pose: Pose) -> Optional[RedTarget]:
        """Synthetic bearing ONLY: cannot satisfy the close-target stop rule."""
        if self.goal_xy is None:
            return None
        delta_x = self.goal_xy[0] - pose[0]
        delta_y = self.goal_xy[1] - pose[1]
        bearing = normalize_angle(math.atan2(delta_y, delta_x) - pose[2])
        offset = -math.tan(max(-1.3, min(1.3, bearing))) / math.tan(
            self.tracking.camera_hfov / 2)
        return RedTarget(offset, 0.0, 0, 0, 0)

    def start_detour(self, pose: Pose, ranges: Sequence[float],
                     angle_min: float, angle_increment: float,
                     range_min: float, range_max: float) -> Decision:
        self.direction = free_side(ranges, angle_min, angle_increment,
                                   range_min, range_max)
        self.start_pose = pose
        self.mode = AVOID_TURN
        return Decision(0.0, self.direction * self.tracking.turn_speed, self.mode)

    def step(self, ranges: Sequence[float], angle_min: float, angle_increment: float,
             range_min: float, range_max: float, target: Optional[RedTarget],
             pose: Optional[Pose]) -> Decision:
        if self.reached:
            return Decision(0.0, 0.0, TARGET_STOP)
        if pose is None:
            return STOP_DECISION

        front = front_distance(ranges, angle_min, angle_increment, range_min,
                               range_max, self.tracking.front_half_angle)
        if front is None:
            return STOP_DECISION

        self.remember_target(target, pose)

        if self.mode == AVOID_TURN:
            assert self.start_pose is not None
            turned = normalize_angle(pose[2] - self.start_pose[2]) * self.direction
            if turned < self.tracking.detour_turn_angle:
                return Decision(0.0, self.direction * self.tracking.turn_speed,
                                AVOID_TURN)
            self.mode = AVOID_DRIVE
            self.start_pose = pose

        if self.mode == AVOID_DRIVE:
            assert self.start_pose is not None
            traveled = math.hypot(pose[0] - self.start_pose[0],
                                  pose[1] - self.start_pose[1])
            if traveled < self.tracking.detour_forward_distance:
                safety_front = corridor_distance(ranges, angle_min, angle_increment,
                                                 range_min, range_max)
                if safety_front is None:
                    return STOP_DECISION
                if safety_front < 0.60:
                    return self.start_detour(pose, ranges, angle_min, angle_increment,
                                             range_min, range_max)
                return Decision(self.tracking.detour_speed, 0.0, AVOID_DRIVE)
            self.mode = SEARCH
            self.start_pose = None

        # If the target goes out of frame during avoidance, turn back toward
        # its odometry-anchored remembered location instead of wandering off.
        guided_target = target if target is not None else self.remembered_target(pose)
        result = choose_velocity(ranges, angle_min, angle_increment,
                                 range_min, range_max, guided_target, self.avoidance,
                                 self.tracking, self.previous_motion, self.reached)
        if result.state == TARGET_STOP:
            self.reached = True
        elif result.state == AVOID:
            return self.start_detour(pose, ranges, angle_min, angle_increment,
                                     range_min, range_max)
        elif guided_target is None:
            self.previous_motion = result.state
        self.mode = result.state
        return result
