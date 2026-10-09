"""Day 5: Control for navigation. Go-to-goal with a P controller, ROS-style
nodes connected by topics, running against CoppeliaSim.

   [SimBridge] --/odom--> [GoToGoal] --/cmd_vel--> [SimBridge]

SimBridge is the "driver" node: the only thing touching the simulator.
Run with CoppeliaSim open + P3DX.
"""
import math

import matplotlib.pyplot as plt

from mini_ros import Bus, Node, spin_once
from sim_helpers import P3DX, wrap


class SimBridge(Node):
    def __init__(self, bus, robot):
        super().__init__("sim_bridge", bus)
        self.r = robot
        self.cmd = {"v": 0.0, "w": 0.0}
        self.pub_odom = self.create_publisher("/odom")
        self.subscribe("/cmd_vel", lambda m: self.cmd.update(m))
        self.create_timer(robot.dt, self.tick)

    def tick(self):
        self.r.drive(self.cmd["v"], self.cmd["w"])
        x, y, th = self.r.true_pose()  # later: replace with your day-3 odometry!
        self.pub_odom({"x": x, "y": y, "th": th})


class GoToGoal(Node):
    K_RHO, K_ALPHA = 0.5, 1.5
    V_MAX, TOL = 0.4, 0.1

    def __init__(self, bus, goals):
        super().__init__("go_to_goal", bus)
        self.goals = list(goals)
        self.pub = self.create_publisher("/cmd_vel")
        self.subscribe("/odom", self.on_odom)
        self.finished = False

    def on_odom(self, o):
        if not self.goals:
            self.pub({"v": 0.0, "w": 0.0})
            self.finished = True
            return
        gx, gy = self.goals[0]
        # TODO 1: rho = distance to goal; alpha = wrap(bearing - o["th"])
        # TODO 2: if rho < TOL: pop goal, self.log it, return
        # TODO 3: v = min(V_MAX, K_RHO*rho) * max(0, cos(alpha))  (slow when facing away)
        #         w = K_ALPHA * alpha
        # TODO 4: self.pub({"v": v, "w": w})
        pass


def main():
    r = P3DX()
    bus = Bus()
    bridge = SimBridge(bus, r)
    ctrl = GoToGoal(bus, [(1.5, 0.0), (1.5, 1.5), (0.0, 1.5), (0.0, 0.0)])
    nodes = [bridge, ctrl]
    path = []
    r.start()
    while not ctrl.finished and r.time() < 90:
        r.step()
        spin_once(nodes, r.time())
        path.append(r.true_pose()[:2])
    r.stop()
    xs, ys = zip(*path)
    plt.plot(xs, ys); plt.axis("equal"); plt.title("Go-to-goal path"); plt.show()


if __name__ == "__main__":
    main()

# STRETCH: tune K_RHO/K_ALPHA; plot rho vs time; add PID on heading.
# STRETCH: goals are in world coordinates -- make them relative to the start pose.
