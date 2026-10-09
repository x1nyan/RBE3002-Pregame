"""Day 10: Capstone - navigate relative waypoints using odometry and sonars.

Run with CoppeliaSim open and a PioneerP3DX in the scene:
    python day10_navigation_capstone.py

Add a few Cuboids to make an obstacle field. The controller uses wheel-encoder
odometry and sonar ranges; ground-truth pose is only recorded for comparison.
"""
import math

import matplotlib.pyplot as plt

from mini_ros import Bus, Node, spin_once
from sim_helpers import AXLE_LENGTH, P3DX, WHEEL_RADIUS, wrap

FRONT = (2, 3, 4, 5)
WAYPOINTS = ((1.5, 0.0), (1.5, 1.5), (0.0, 1.5), (0.0, 0.0))
OBSTACLE_DISTANCE = 0.4


class EncoderOdometry:
    def __init__(self, wheel_angles: tuple[float, float]):
        self.previous = wheel_angles
        self.x = self.y = self.theta = 0.0

    def update(self, wheel_angles: tuple[float, float]) -> tuple[float, float, float]:
        left, right = wheel_angles
        previous_left, previous_right = self.previous
        dl = wrap(left - previous_left) * WHEEL_RADIUS
        dr = wrap(right - previous_right) * WHEEL_RADIUS
        self.previous = wheel_angles

        distance = (dl + dr) / 2.0
        turn = (dr - dl) / AXLE_LENGTH
        midpoint_heading = self.theta + turn / 2.0
        self.x += distance * math.cos(midpoint_heading)
        self.y += distance * math.sin(midpoint_heading)
        self.theta = wrap(self.theta + turn)
        return self.x, self.y, self.theta


class SimBridge(Node):
    """The only node that reads from or writes to CoppeliaSim."""

    def __init__(self, bus: Bus, robot: P3DX):
        super().__init__("sim_bridge", bus)
        self.robot = robot
        self.command = {"v": 0.0, "w": 0.0}
        self.odometry = EncoderOdometry(robot.wheel_angles())
        self.publish_odom = self.create_publisher("/odom")
        self.publish_ranges = self.create_publisher("/ranges")
        self.subscribe("/cmd_vel", self.command.update)
        self.create_timer(robot.dt, self.tick)

    def tick(self) -> None:
        self.robot.drive(self.command["v"], self.command["w"])
        self.odometry.update(self.robot.wheel_angles())
        self.publish_odom({"x": self.odometry.x,
                            "y": self.odometry.y,
                            "th": self.odometry.theta})
        self.publish_ranges(self.robot.ranges())


class WaypointNavigator(Node):
    def __init__(self, bus: Bus):
        super().__init__("waypoint_navigator", bus)
        self.goals = list(WAYPOINTS)
        self.odom = None
        self.ranges = [1.0] * 16
        self.finished = False
        self.publish_command = self.create_publisher("/cmd_vel")
        self.subscribe("/odom", self.on_odom)
        self.subscribe("/ranges", self.on_ranges)

    def on_odom(self, odom: dict[str, float]) -> None:
        self.odom = odom
        self.control()

    def on_ranges(self, ranges: list[float]) -> None:
        self.ranges = ranges
        self.control()

    def control(self) -> None:
        if self.finished:
            self.publish_command({"v": 0.0, "w": 0.0})
            return
        if self.odom is None:
            return

        if not self.goals:
            self.finished = True
            self.publish_command({"v": 0.0, "w": 0.0})
            self.log("All waypoints reached")
            return

        x, y, theta = self.odom["x"], self.odom["y"], self.odom["th"]
        gx, gy = self.goals[0]
        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)
        if distance < 0.12:
            reached = self.goals.pop(0)
            self.log(f"Reached waypoint {reached}")
            self.control()
            return

        heading_error = wrap(math.atan2(dy, dx) - theta)
        front = min(self.ranges[index] for index in FRONT)
        if front < OBSTACLE_DISTANCE:
            left_clearance = min(self.ranges[index] for index in (0, 1, 2))
            right_clearance = min(self.ranges[index] for index in (5, 6, 7))
            turn = 0.8 if left_clearance >= right_clearance else -0.8
            self.publish_command({"v": 0.0, "w": turn})
            return

        velocity = min(0.35, 0.6 * distance) * max(0.0, math.cos(heading_error))
        angular = max(-1.2, min(1.2, 2.0 * heading_error))
        self.publish_command({"v": velocity, "w": angular})


def main() -> None:
    robot = P3DX()
    bus = Bus()
    bridge = SimBridge(bus, robot)
    navigator = WaypointNavigator(bus)
    nodes = [bridge, navigator]
    estimated_path = []
    true_path = []

    robot.start()
    try:
        while not navigator.finished and robot.time() < 180.0:
            robot.step()
            spin_once(nodes, robot.time())
            estimated_path.append(
                (bridge.odometry.x, bridge.odometry.y)
            )
            true_path.append(robot.true_pose()[:2])
    finally:
        robot.stop()

    if not navigator.finished:
        print("Time limit reached before all waypoints were completed.")
    if estimated_path:
        estimated_x, estimated_y = zip(*estimated_path)
        true_x, true_y = zip(*true_path)
        plt.plot(estimated_x, estimated_y, label="encoder odometry")
        plt.plot(true_x, true_y, "--", label="ground truth (validation)")
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Day 10 navigation capstone")
        plt.legend()
        plt.show()


if __name__ == "__main__":
    main()

# Exercises:
# 1. Put obstacles on the route and test whether the sonar turn rule gets stuck.
# 2. Adjust waypoint and obstacle thresholds, recording the outcome.
# 3. Add command delay/drop from day8 and compare estimated vs. true paths.
