"""Tiny ROS-like framework (topics, nodes, timers) so you can practice ROS
patterns on Windows without installing ROS. Concepts map 1:1:

    Bus.publish / Node.create_publisher  ->  rospy/rclpy publishers
    Node.subscribe                       ->  subscriptions + callbacks
    Node.create_timer                    ->  timers
    spin_once(nodes, t)                  ->  rclpy.spin / executor

Messages are plain dicts. Single-threaded and deterministic:
call spin_once() each simulation step.
"""
from __future__ import annotations

from collections import defaultdict
from typing import Any, Callable


class Bus:
    def __init__(self) -> None:
        self._subs: dict[str, list[Callable[[Any], None]]] = defaultdict(list)
        self.latest: dict[str, Any] = {}

    def publish(self, topic: str, msg: Any) -> None:
        self.latest[topic] = msg
        for cb in list(self._subs[topic]):
            cb(msg)

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        self._subs[topic].append(cb)


class Node:
    def __init__(self, name: str, bus: Bus) -> None:
        self.name = name
        self.bus = bus
        self._timers: list[list] = []  # [period, next_time, callback]
        self.now = 0.0

    def create_publisher(self, topic: str) -> Callable[[Any], None]:
        return lambda msg: self.bus.publish(topic, msg)

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        self.bus.subscribe(topic, cb)

    def create_timer(self, period: float, cb: Callable[[], None]) -> None:
        self._timers.append([period, period, cb])

    def log(self, text: str) -> None:
        print(f"[{self.now:7.2f}] [{self.name}] {text}")

    def _tick(self, t: float) -> None:
        self.now = t
        for timer in self._timers:
            if t + 1e-9 >= timer[1]:
                timer[1] += timer[0]
                timer[2]()


def spin_once(nodes: list[Node], t: float) -> None:
    for n in nodes:
        n._tick(t)
