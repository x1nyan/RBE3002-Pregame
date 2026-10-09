"""Day 3: Dead reckoning in CoppeliaSim.
Estimate pose from wheel encoders only, compare against ground truth, plot error.

Goal: drive a 2 m square (open loop, timed) and see how odometry drifts.
Run with CoppeliaSim open + P3DX in scene.
"""
import math

import matplotlib.pyplot as plt

from sim_helpers import AXLE_LENGTH, WHEEL_RADIUS, P3DX, wrap


class Odometry:
    def __init__(self, x=0.0, y=0.0, th=0.0):
        self.x, self.y, self.th = x, y, th
        self.prev = None

    def update(self, ang_l, ang_r):
        """ang_* are wheel angles (rad). Update self.x/y/th."""
        if self.prev is None:
            self.prev = (ang_l, ang_r)
            return
        # TODO 1: wheel angle deltas (use wrap() -- joint angles are cyclic!)
        # TODO 2: dsl, dsr = distance each wheel travelled (WHEEL_RADIUS * dangle)
        # TODO 3: ds = (dsl+dsr)/2 ; dth = (dsr-dsl)/AXLE_LENGTH
        # TODO 4: midpoint update: x += ds*cos(th+dth/2), y += ds*sin(th+dth/2), th += dth
        self.prev = (ang_l, ang_r)


def main():
    r = P3DX()
    r.start()
    r.step()
    x0, y0, th0 = r.true_pose()
    odo = Odometry(x0, y0, th0)

    est, truth = [], []
    # TODO 5: build `plan` as a list of (v, w, duration_s) for a 2 m square
    #         (forward 10 s @0.2 m/s; turn 90 deg: w=0.5 rad/s -> (pi/2)/0.5 s)
    plan = []
    for v, w, dur in plan:
        t_end = r.time() + dur
        while r.time() < t_end:
            r.drive(v, w)
            r.step()
            odo.update(*r.wheel_angles())
            est.append((odo.x, odo.y))
            truth.append(r.true_pose()[:2])
    r.stop()

    if not est:
        print("Fill in the TODOs first.")
        return
    ex, ey = zip(*est)
    tx, ty = zip(*truth)
    plt.plot(tx, ty, label="ground truth")
    plt.plot(ex, ey, "--", label="odometry")
    plt.axis("equal")
    plt.legend()
    plt.title("Dead reckoning")
    plt.show()
    print("final position error [m]:", math.hypot(ex[-1] - tx[-1], ey[-1] - ty[-1]))


if __name__ == "__main__":
    main()

# STRETCH: add Gaussian noise to the encoder angles (np.random.normal) and
#          run 20 trials; plot a histogram of final position error.
# THINK: which error (wheel radius vs axle length) hurts heading most?
