"""Day 9: Tune a go-to-goal controller in a simple unicycle simulation.

Run: python day9_controller_tuning.py

This lets you compare gains quickly before trying them on the simulated robot.
"""
import math

DT = 0.05
GOAL = (2.0, 1.0)
TOLERANCE = 0.05
MAX_STEPS = 2000


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def simulate(k_rho: float, k_alpha: float) -> tuple[bool, int, float, float]:
    x, y, theta = 0.0, 0.0, 0.0
    path_length = 0.0
    for step in range(MAX_STEPS):
        dx, dy = GOAL[0] - x, GOAL[1] - y
        rho = math.hypot(dx, dy)
        if rho < TOLERANCE:
            return True, step, rho, path_length

        alpha = wrap(math.atan2(dy, dx) - theta)
        v = min(0.5, k_rho * rho) * max(0.0, math.cos(alpha))
        w = max(-1.5, min(1.5, k_alpha * alpha))

        theta = wrap(theta + w * DT)
        distance = v * DT
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)
        path_length += distance

    error = math.hypot(GOAL[0] - x, GOAL[1] - y)
    return False, MAX_STEPS, error, path_length


def main() -> None:
    settings = [(0.3, 1.0), (0.8, 2.0), (1.5, 4.0)]
    for k_rho, k_alpha in settings:
        converged, steps, error, distance = simulate(k_rho, k_alpha)
        print(
            f"K_rho={k_rho:.1f}, K_alpha={k_alpha:.1f}: "
            f"converged={converged}, time={steps * DT:.1f}s, "
            f"final error={error:.3f}m, path={distance:.2f}m"
        )
    assert all(simulate(*gains)[0] for gains in settings)


if __name__ == "__main__":
    main()

# Exercises:
# 1. Test one gain at a time and record its effect on convergence time.
# 2. Change the start heading or goal and find gains that still converge.
# 3. Add a plot of the trajectory and compare path length with straight-line distance.
