"""Day 2: ROS-style architecture with mini_ros (no simulator needed).
Three nodes talking over topics, like a real ROS graph:

   [SquarePlanner] --/cmd_vel--> [FakeRobot] --/odom--> [Logger]

Run: python day2_ros_nodes.py
"""
import math

from mini_ros import Bus, Node, spin_once


class FakeRobot(Node):
    """Subscribes /cmd_vel {v,w}; integrates a unicycle; publishes /odom at 10 Hz."""

    def __init__(self, bus):
        super().__init__("fake_robot", bus)
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.pub_odom = self.create_publisher("/odom")
        self.subscribe("/cmd_vel", self.on_cmd)
        self.create_timer(0.01, self.integrate)
        self.create_timer(0.1, self.publish_odom)

    def on_cmd(self, msg):
        # TODO: store msg["v"], msg["w"]
        pass

    def integrate(self):
        dt = 0.01
        # TODO: Euler-integrate x, y, th using self.v, self.w
        pass

    def publish_odom(self):
        # TODO: publish {"x":..., "y":..., "th":...} to /odom
        pass


class SquarePlanner(Node):
    """Drive a 1 m square using /odom feedback."""

    def __init__(self, bus):
        super().__init__("square_planner", bus)
        self.pub = self.create_publisher("/cmd_vel")
        self.subscribe("/odom", self.on_odom)
        self.odom = None
        self.state = "FORWARD"
        self.corner = None
        self.corner_th = 0.0
        self.corners_done = 0
        self.create_timer(0.1, self.tick)

    def on_odom(self, msg):
        self.odom = msg
        if self.corner is None:
            self.corner = (msg["x"], msg["y"])
            self.corner_th = msg["th"]

    def tick(self):
        if self.odom is None:
            return
        # TODO: implement the FORWARD / TURN state machine.
        #  FORWARD: publish {"v":0.2,"w":0}; when distance from self.corner >= 1.0 -> TURN
        #  TURN:    publish {"v":0,"w":0.5};  when heading change >= pi/2 -> FORWARD,
        #           update self.corner/self.corner_th, corners_done += 1
        #  After 4 corners publish zeros and self.log("done").
        #  Wrap the heading difference to (-pi, pi]!
        pass


class Logger(Node):
    def __init__(self, bus):
        super().__init__("logger", bus)
        self.create_timer(2.0, self.report)

    def report(self):
        m = self.bus.latest.get("/odom")
        if m:
            self.log(f"x={m['x']:.2f} y={m['y']:.2f} th={math.degrees(m['th']):.0f}deg")


if __name__ == "__main__":
    bus = Bus()
    nodes = [FakeRobot(bus), SquarePlanner(bus), Logger(bus)]
    t = 0.0
    while t < 60:
        t = round(t + 0.01, 6)
        spin_once(nodes, t)

# STRETCH: add a /scan topic and a SafetyNode that overrides /cmd_vel with
# zeros when an obstacle is too close (use a separate /cmd_vel_safe topic).
