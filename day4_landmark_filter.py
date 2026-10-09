"""Day 4: Landmark updates / sensor fusion (pure Python, no simulator).
1D robot on a line moving at ~1 m/s. Odometry drifts; a landmark range sensor
corrects it. Implement a Kalman filter and compare to dead reckoning.

Run: python day4_landmark_filter.py
"""
import numpy as np
import matplotlib.pyplot as plt

rng = np.random.default_rng(0)
DT, STEPS = 0.1, 300
LANDMARK = 50.0              # known position
Q = 0.05 ** 2                # process noise variance per step (odometry error)
R = 0.5 ** 2                 # range sensor noise variance
MEAS_EVERY = 10              # landmark measured every 1 s, only once x > 10


class KF1D:
    def __init__(self, x0=0.0, p0=1.0):
        self.x, self.p = x0, p0

    def predict(self, u):
        """u = odometry displacement this step."""
        # TODO: x += u ; p += Q
        pass

    def update(self, z_range):
        """z_range = measured LANDMARK - x. Update x, p."""
        # TODO: predicted measurement h = LANDMARK - x (Jacobian H = -1)
        #       innovation y = z - h ; S = H*p*H + R ; K = p*H/S
        #       x += K*y ; p = (1 - K*H) * p
        pass


def simulate():
    true_x, dr_x = 0.0, 0.0
    kf = KF1D()
    log = {"true": [], "dr": [], "kf": []}
    for k in range(STEPS):
        u_cmd = 1.0 * DT
        true_x += u_cmd * 1.02                          # real robot is 2% faster (slip/calibration)
        odo = u_cmd + rng.normal(0, np.sqrt(Q))         # odometry only knows the command
        dr_x += odo
        kf.predict(odo)
        if k % MEAS_EVERY == 0 and true_x > 10:
            z = (LANDMARK - true_x) + rng.normal(0, np.sqrt(R))
            kf.update(z)
        log["true"].append(true_x); log["dr"].append(dr_x); log["kf"].append(kf.x)
    return log


if __name__ == "__main__":
    log = simulate()
    t = np.arange(STEPS) * DT
    plt.plot(t, np.array(log["dr"]) - log["true"], label="dead reckoning error")
    plt.plot(t, np.array(log["kf"]) - log["true"], label="Kalman error")
    plt.xlabel("t [s]"); plt.ylabel("position error [m]"); plt.legend(); plt.show()

# STRETCH 1: extend to 2D (state x,y,theta) with range+bearing to landmarks (EKF).
# STRETCH 2: in CoppeliaSim, place a cylinder landmark, compute range/bearing from
#            true_pose() + noise, and fuse with your day-3 odometry.
