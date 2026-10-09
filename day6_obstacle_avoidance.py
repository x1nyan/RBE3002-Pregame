"""Day 6: Sensing in an unknown environment: obstacle avoidance.
Add a few Cuboids to the scene around the robot's path. Use the 16 sonars.

P3DX sonar indices: 0..7 sweep left->right across the front (3,4 = front pair),
8..15 face backwards. Print r.ranges() once to confirm on your model.

Implement in order:
 1. Reactive stop:  stop if any front sonar < 0.3 m.
 2. Braitenberg:    wheel speeds = base + weighted sum of detections.
 3. Wall follower:  keep ~0.4 m from a wall on the right with a P controller
                    on the right-side sonar (index 7).
"""
from sim_helpers import P3DX

FRONT = [2, 3, 4, 5]
BRAITENBERG_L = [-0.2, -0.6, -1.0, -1.4, 1.4, 1.0, 0.6, 0.2]  # sonars 0..7 -> left wheel
BRAITENBERG_R = [0.2, 0.6, 1.0, 1.4, -1.4, -1.0, -0.6, -0.2]  # sonars 0..7 -> right wheel
V0 = 2.0  # rad/s base wheel speed


def braitenberg(ranges, max_range=1.0):
    # TODO: detect_i = 1 - ranges[i]/max_range  (0 if nothing seen)
    #       wl = V0 + sum(BRAITENBERG_L[i]*detect_i) ; same for wr
    #       return wl, wr
    return V0, V0


def wall_follow(ranges, target=0.4):
    # TODO: error = target - ranges[7]; w = k * error ; return (v, w)
    return 0.2, 0.0


def main():
    r = P3DX()
    r.start()
    print("Sonar sample:", [round(d, 2) for d in r.ranges()])
    while r.time() < 60:
        wl, wr = braitenberg(r.ranges())
        r.set_wheels(wl, wr)
        r.step()
    r.stop()


if __name__ == "__main__":
    main()

# STRETCH: combine go-to-goal (day 5) + avoidance: goal attracts, obstacles repel
#          (behavior-based / subsumption).
