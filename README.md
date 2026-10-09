# RBE navigation warm-up: Python + ROS-style code + CoppeliaSim

Setup done: Python 3.13 with `coppeliasim-zmqremoteapi-client`, `numpy`, `matplotlib`.
Run from this folder: `python day0_hello_sim.py`.

**Scene:** open CoppeliaSim, add `Robots > Mobile > PioneerP3DX` and a floor. Scripts
start/stop the simulation themselves. If object lookups fail, check the names in your scene
hierarchy against `sim_helpers.py`.

`mini_ros.py` is a tiny stand-in for ROS (nodes, topics, timers) so you learn the pattern:
pub/sub, decoupled nodes, a hardware-bridge node. The structure translates directly to `rclpy`.

## Week plan (~1-2 h/day)

| Day | File | Skills | Done when |
|---|---|---|---|
| 0 | `day0_hello_sim.py` | Connect, step mode, circle driving | Robot drives a r=0.5 m circle |
| 1 | `day1_python_warmup.py` | dataclasses, numpy, angle wrap, kinematics | All asserts pass |
| 2 | `day2_ros_nodes.py` | Nodes, pub/sub, timers, state machines | Fake robot traces a 1 m square |
| 3 | `day3_dead_reckoning.py` | Encoders, odometry, drift, plotting | Odom vs truth plot + error |
| 4 | `day4_landmark_filter.py` | Landmark update, Kalman filter | KF error << dead-reckoning error |
| 5 | `day5_go_to_goal.py` | P control, bridge node | Visits 4 waypoints |
| 6 | `day6_obstacle_avoidance.py` | Range sensing, Braitenberg, wall following | Avoids boxes for 60 s |
| 7 | Mini project | Integrate | See below |

## Day 7 mini project
Navigate waypoints through an obstacle field using **your odometry (day 3) + landmark Kalman
update (day 4) + go-to-goal with avoidance (days 5-6)**, each as its own node on the bus. Plot
estimated vs true pose error. Stretch: simulate comms problems on `/cmd_vel` (drop 20% of
messages, 0.3 s delay) and see how control degrades -- ties to the course's remote-control and
wireless topics.

## Tips
- Never feed `true_pose()` to your controller in the end; it's for validation only.
- Try each TODO yourself first; ask me for hints or reference solutions for any day.
