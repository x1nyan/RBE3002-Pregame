"""Day 8: Explore how dropped and delayed velocity commands affect a robot.

Run: python day8_command_network.py

This deterministic toy channel models a command topic with fixed latency,
random packet loss, and a zero-order hold of the last delivered command.
"""
from dataclasses import dataclass
import heapq
import random


@dataclass(frozen=True)
class Command:
    sent_at: int
    velocity: float


class CommandChannel:
    def __init__(self, latency_steps: int, drop_probability: float, seed: int = 0):
        if latency_steps < 0:
            raise ValueError("latency_steps must be non-negative")
        if not 0.0 <= drop_probability <= 1.0:
            raise ValueError("drop_probability must be between 0 and 1")
        self.latency_steps = latency_steps
        self.drop_probability = drop_probability
        self.rng = random.Random(seed)
        self.pending: list[tuple[int, int, Command]] = []
        self.sequence = 0
        self.last_delivered = Command(sent_at=-1, velocity=0.0)
        self.dropped = 0

    def step(self, step: int, velocity: float) -> Command:
        """Send one command, deliver anything due, and return the held command."""
        command = Command(sent_at=step, velocity=velocity)
        if self.rng.random() < self.drop_probability:
            self.dropped += 1
        else:
            deliver_at = step + self.latency_steps
            heapq.heappush(
                self.pending, (deliver_at, self.sequence, command)
            )
            self.sequence += 1

        while self.pending and self.pending[0][0] <= step:
            _, _, self.last_delivered = heapq.heappop(self.pending)
        return self.last_delivered


def run_experiment(latency_steps: int, drop_probability: float) -> tuple[int, float]:
    channel = CommandChannel(latency_steps, drop_probability, seed=7)
    commands = [0.3] * 10 + [0.0] * 5 + [-0.2] * 10 + [0.0] * 5
    total_tracking_error = 0.0
    for step, requested in enumerate(commands):
        received = channel.step(step, requested)
        total_tracking_error += abs(received.velocity - requested)
    return channel.dropped, total_tracking_error / len(commands)


def main() -> None:
    baseline = run_experiment(latency_steps=0, drop_probability=0.0)
    impaired = run_experiment(latency_steps=2, drop_probability=0.2)
    print(f"Perfect channel: dropped={baseline[0]}, mean velocity error={baseline[1]:.3f}")
    print(f"Impaired channel: dropped={impaired[0]}, mean velocity error={impaired[1]:.3f}")
    assert baseline == (0, 0.0)
    assert impaired[0] > 0 and impaired[1] > 0


if __name__ == "__main__":
    main()

# Exercises:
# 1. Change the random seed and compare packet-loss patterns.
# 2. Try latency_steps=5 and drop_probability=0.0. Explain the tracking error.
# 3. Add a maximum command age so stale commands are discarded.
