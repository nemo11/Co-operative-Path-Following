#!/usr/bin/env python3
"""Basic LQR path following demonstration.

Port of ``path_following.m`` — the simplest example showing LQR control
for a single-state path following system.
"""

import numpy as np
from scipy.signal import lsim
from scipy.linalg import solve_continuous_are
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from cooperative_path_following.lqr import lqr


def main():
    # Simple double-integrator dynamics: [d, d_dot]
    A = np.array([[0.0, 1.0],
                   [0.0, 0.0]])
    B = np.array([[0.0],
                   [1.0]])
    C = np.eye(2)
    D = np.zeros((2, 1))

    Q = np.eye(2)
    R = np.array([[1.0]])
    K = lqr(A, B, Q, R)
    print(f"LQR gain K = {K}")

    # Closed-loop system
    Ac = A - B @ K
    Bc = B
    Cc = C
    Dc = D

    # Simulation
    t = np.arange(0, 10.01, 0.01)
    r = 0.2 * np.ones((len(t), 1))
    X0 = np.array([0.5, 0.5])

    from scipy.signal import StateSpace
    sys_cl = StateSpace(Ac, Bc, Cc, Dc)
    tout, y, x = lsim(sys_cl, r, t, X0=X0)

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twinx()
    ax1.plot(tout, y[:, 0], "b-", label="Position Error")
    ax2.plot(tout, y[:, 1], "r-", label="Velocity Error")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position Error", color="b")
    ax2.set_ylabel("Velocity Error", color="r")
    ax1.set_title("Step Response with LQR Control")
    fig.legend(loc="upper right", bbox_to_anchor=(0.9, 0.9))
    fig.savefig("path_following_lqr.png", dpi=150, bbox_inches="tight")
    print("Plot saved to path_following_lqr.png")


if __name__ == "__main__":
    main()
