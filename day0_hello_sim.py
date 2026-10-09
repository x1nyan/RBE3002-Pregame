"""Day 0: sanity check. Open CoppeliaSim, add a PioneerP3DX + floor, then run.
Expected: robot drives forward ~3 s, prints poses, then stops."""
from sim_helpers import P3DX

r = P3DX()
r.start()
while r.time() < 3.0:
    r.drive(0.2, 0.0)
    r.step()
    if round(r.time() / r.dt) % 10 == 0:
        print(f"t={r.time():.2f} true pose={r.true_pose()}")
r.stop()

# TODO: change drive() so the robot moves in a circle of radius 0.5 m.
#       (hint: v = w * radius)
