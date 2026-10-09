# RBE navigation warm-up: Python + ROS-style code + CoppeliaSim

Setup done: Python 3.13 with `coppeliasim-zmqremoteapi-client`, `numpy`, `matplotlib`.
Before the plan, run `python day0_hello_sim.py` to confirm the simulator connection.

**Scene:** open CoppeliaSim, add `Robots > Mobile > PioneerP3DX` and a floor. Scripts
start/stop the simulation themselves. If object lookups fail, check the names in your scene
hierarchy against `sim_helpers.py`.

`mini_ros.py` is a tiny stand-in for ROS (nodes, topics, timers) so you learn the pattern:
pub/sub, decoupled nodes, a hardware-bridge node. The structure translates directly to `rclpy`.

## 10-day plan (~1-2 h/day)

| Day | File / focus | Skills | Done when |
|---|---|---|---|
| 1 | `day1_python_warmup.py` | Dataclasses, numpy, angle wrap, kinematics | All asserts pass |
| 2 | `day2_ros_nodes.py` | Nodes, pub/sub, timers, state machines | Fake robot traces a 1 m square |
| 3 | `day3_dead_reckoning.py` | Encoders, odometry, drift, plotting | Odom vs. truth plot and error |
| 4 | `day4_landmark_filter.py` | Landmark updates, Kalman filter | KF error is lower than dead-reckoning error |
| 5 | `day5_go_to_goal.py` | P control, bridge node | Visits 4 waypoints |
| 6 | `day6_obstacle_avoidance.py` | Range sensing, Braitenberg, wall following | Avoids boxes for 60 s |
| 7 | `day7_grid_path_planning.py` | Occupancy grids, A* search | Finds a valid shortest path around walls |
| 8 | `day8_command_network.py` | Command latency, packet loss, zero-order hold | Impairments increase tracking error |
| 9 | `day9_controller_tuning.py` | Gain tuning, convergence, path length | All tested gains reach the goal |
| 10 | `day10_navigation_capstone.py` | Encoder odometry, waypoints, sonar avoidance, integration | Completes the waypoint course and plots estimated vs. true path |

Day 0 (`day0_hello_sim.py`) is setup and does not count toward the ten study days.
Days 1-9 build up the navigation techniques; Day 10 combines them in a CoppeliaSim
capstone. For the capstone, add a few Cuboids around the route to create an obstacle field.

## Tips
- Never feed `true_pose()` to your controller in the end; it's for validation only.
- Try each TODO yourself first; ask me for hints or reference solutions for any day.
- Run the new standalone exercises with `python day7_grid_path_planning.py`,
  `python day8_command_network.py`, and `python day9_controller_tuning.py`.
