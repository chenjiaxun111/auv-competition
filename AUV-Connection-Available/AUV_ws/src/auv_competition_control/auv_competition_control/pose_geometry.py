"""Pure geometry helpers shared by gate Pose runtime code and unit tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


@dataclass(frozen=True)
class GateImageGeometry:
    center_x_norm: float
    center_y_norm: float
    width_norm: float
    height_norm: float
    bearing_x_rad: float
    in_bounds: bool
    convex: bool
    ordered: bool
    area_norm: float

    @property
    def valid(self) -> bool:
        return (
            self.in_bounds
            and self.convex
            and self.ordered
            and self.area_norm > 0.002
        )


def _cross(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - b[1]) - (
        b[1] - a[1]) * (c[0] - b[0])


def gate_image_geometry(
    points: Sequence[Sequence[float]],
    image_width: int,
    image_height: int,
    fx: float,
    cx: float,
) -> GateImageGeometry:
    """Validate TL,TR,BR,BL points and produce normalized servo features."""
    if len(points) != 4 or image_width <= 0 or image_height <= 0 or fx <= 0.0:
        raise ValueError('four points and positive image/intrinsic sizes required')
    values: Tuple[Tuple[float, float], ...] = tuple(
        (float(point[0]), float(point[1])) for point in points)
    if not all(math.isfinite(value) for point in values for value in point):
        raise ValueError('keypoints must be finite')

    tl, tr, br, bl = values
    in_bounds = all(
        0.0 <= point[0] < image_width and 0.0 <= point[1] < image_height
        for point in values
    )
    crosses = [_cross(values[i], values[(i + 1) % 4], values[(i + 2) % 4])
               for i in range(4)]
    convex = all(value > 0.0 for value in crosses) or all(
        value < 0.0 for value in crosses)
    ordered = (
        (tl[0] + bl[0]) < (tr[0] + br[0])
        and (tl[1] + tr[1]) < (bl[1] + br[1])
    )
    area_px = 0.5 * abs(sum(
        values[index][0] * values[(index + 1) % 4][1]
        - values[(index + 1) % 4][0] * values[index][1]
        for index in range(4)
    ))
    centre_x = sum(point[0] for point in values) / 4.0
    centre_y = sum(point[1] for point in values) / 4.0
    top = math.dist(tl, tr)
    bottom = math.dist(bl, br)
    left = math.dist(tl, bl)
    right = math.dist(tr, br)
    return GateImageGeometry(
        center_x_norm=2.0 * (centre_x / image_width - 0.5),
        center_y_norm=2.0 * (centre_y / image_height - 0.5),
        width_norm=(top + bottom) / (2.0 * image_width),
        height_norm=(left + right) / (2.0 * image_height),
        bearing_x_rad=math.atan2(centre_x - cx, fx),
        in_bounds=in_bounds,
        convex=convex,
        ordered=ordered,
        area_norm=area_px / (image_width * image_height),
    )
