"""CoppeliaSim helpers. Scene: add  Robots > Mobile > PioneerP3DX  (and a floor).
Start CoppeliaSim first; it listens on ZMQ port 23000 by default.
"""
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

WHEEL_RADIUS = 0.0975  # P3DX
AXLE_LENGTH = 0.381    # P3DX wheel separation


class P3DX:
    def __init__(self, stepping: bool = True, robot: str = "/PioneerP3DX"):
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.sim.setStepping(stepping)
        self.body = self.sim.getObject(robot)
        self.left = self.sim.getObject(f"{robot}/leftMotor")
        self.right = self.sim.getObject(f"{robot}/rightMotor")
        self.sonars = [self.sim.getObject(f"{robot}/ultrasonicSensor", {"index": i})
                       for i in range(16)]
        self.dt = self.sim.getSimulationTimeStep()

    # --- run control
    def start(self):
        self.sim.startSimulation()

    def step(self):
        self.client.step()

    def time(self) -> float:
        return self.sim.getSimulationTime()

    def stop(self):
        self.set_wheels(0, 0)
        self.sim.stopSimulation()

    # --- actuation: v [m/s], w [rad/s]
    def set_wheels(self, wl: float, wr: float):
        self.sim.setJointTargetVelocity(self.left, wl)
        self.sim.setJointTargetVelocity(self.right, wr)

    def drive(self, v: float, w: float):
        wl = (v - w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        wr = (v + w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        self.set_wheels(wl, wr)

    # --- sensing
    def wheel_angles(self):
        return (self.sim.getJointPosition(self.left),
                self.sim.getJointPosition(self.right))

    def true_pose(self):
        """Ground truth (x, y, yaw). Use only to CHECK your estimates."""
        x, y, _ = self.sim.getObjectPosition(self.body, self.sim.handle_world)
        qx, qy, qz, qw = self.sim.getObjectQuaternion(self.body, self.sim.handle_world)
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return x, y, yaw

    def ranges(self, max_range: float = 1.0):
        """16 sonar distances (max_range if nothing detected)."""
        out = []
        for h in self.sonars:
            res, dist, *_ = self.sim.readProximitySensor(h)
            out.append(dist if res > 0 else max_range)
        return out


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))
