"""Detect red diseased plants in BGR camera frames, without ROS imports."""
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class RedTarget:
    """Horizontal offset (-1 left, +1 right), contour area, centroid and width."""
    offset: float
    area: float
    cx: int
    cy: int
    width: int = 0


def find_red_target(bgr: np.ndarray, min_area: float = 120.0) -> Optional[RedTarget]:
    """Find the largest saturated red blob; reject brown pots and image noise."""
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.size == 0:
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower = cv2.inRange(hsv, np.array([0, 110, 75]), np.array([7, 255, 255]))
    upper = cv2.inRange(hsv, np.array([170, 110, 75]), np.array([179, 255, 255]))
    mask = cv2.bitwise_or(lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return None
    moments = cv2.moments(contour)
    if moments['m00'] <= 0:
        return None
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    half = max(1.0, (bgr.shape[1] - 1) / 2.0)
    offset = float(np.clip((cx - half) / half, -1.0, 1.0))
    _, _, width, _ = cv2.boundingRect(contour)
    return RedTarget(offset, area, cx, cy, width)
