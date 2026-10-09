"""Day 1: Python warm-up (no simulator). Fill in each TODO, then run:
    python day1_python_warmup.py
Every assert must pass."""
import math
from dataclasses import dataclass

import numpy as np


# 1. Dataclasses + methods ---------------------------------------------------
@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def distance_to(self, other: "Pose") -> float:
        # TODO: Euclidean distance
        raise NotImplementedError

    def bearing_to(self, other: "Pose") -> float:
        # TODO: angle from self to other in the WORLD frame (atan2)
        raise NotImplementedError


# 2. Angle wrapping ----------------------------------------------------------
def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    # TODO
    raise NotImplementedError


# 3. Unicycle (differential drive) kinematics ---------------------------------
def step_unicycle(p: Pose, v: float, w: float, dt: float) -> Pose:
    """x' = v cos(theta), y' = v sin(theta), theta' = w. Use Euler integration."""
    # TODO
    raise NotImplementedError


# 4. Wheel speeds <-> (v, w) --------------------------------------------------
def wheels_to_twist(vl: float, vr: float, axle: float):
    """Return (v, w) from left/right wheel LINEAR speeds."""
    # TODO
    raise NotImplementedError


# 5. NumPy: rotate points between frames --------------------------------------
def robot_to_world(points_robot: np.ndarray, pose: Pose) -> np.ndarray:
    """points_robot: (N,2) array in robot frame -> (N,2) in world frame.
    Do it with a 2x2 rotation matrix, no Python loops."""
    # TODO
    raise NotImplementedError


# 6. Comprehensions / sorting -------------------------------------------------
def closest_landmark(p: Pose, landmarks: dict[str, tuple[float, float]]) -> str:
    """Return the name of the closest landmark. One-liner with min(key=...)."""
    # TODO
    raise NotImplementedError


if __name__ == "__main__":
    a, b = Pose(0, 0, 0), Pose(3, 4, 0)
    assert math.isclose(a.distance_to(b), 5.0)
    assert math.isclose(a.bearing_to(Pose(0, 1)), math.pi / 2)
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(abs(wrap_angle(-3 * math.pi)), math.pi)

    p = Pose()
    for _ in range(1000):  # circle: v=1, w=1 for 2*pi seconds
        p = step_unicycle(p, 1.0, 1.0, 2 * math.pi / 1000)
    assert abs(p.x) < 0.05 and abs(p.y) < 0.05, p  # back near start (Euler drift)

    v, w = wheels_to_twist(0.4, 0.6, 0.5)
    assert math.isclose(v, 0.5) and math.isclose(w, 0.4)

    pts = robot_to_world(np.array([[1.0, 0.0], [0.0, 1.0]]), Pose(1, 1, math.pi / 2))
    assert np.allclose(pts, [[1, 2], [0, 1]])

    assert closest_landmark(Pose(), {"A": (5, 5), "B": (1, 1)}) == "B"
    print("Day 1 all passed")

# STRETCH: change step_unicycle to exact arc integration (when w != 0) and
# compare drift against Euler with a big dt (e.g. 0.5 s).
