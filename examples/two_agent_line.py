#!/usr/bin/env python3
"""Two agents following line paths with time coordination.

Port of ``time_coordination_line_simulation.m``.
"""

import numpy as np
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.visualization import plot_agents


def main():
    total_time = 9.0
    total_distance = 20.0
    v1_dash = total_distance / total_time
    v2_dash = total_distance / total_time

    # State-space model: [gamma, d1, Vd1, d2, Vd2]
    A = np.zeros((5, 5))
    A[1, 2] = 1.0
    A[3, 4] = 1.0

    B = np.array([
        [0.0, 0.0, 1.0, -1.0],
        [0.0, 0.0, 0.0, 0.0],
        [v1_dash, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, v2_dash, 0.0, 0.0],
    ])

    Q = np.eye(5)
    R = np.eye(4)
    K = lqr(A, B, Q, R)

    print(f"LQR gain K shape: {K.shape}")

    dt = 0.01
    total_steps = int(total_time / dt)

    # Initial conditions — agents at (0,2) and (0,4)
    target_y1 = 2.0
    target_y2 = 4.0
    a1 = Agent(x=0.0, y=target_y1, si=0.0)
    a2 = Agent(x=0.0, y=target_y2, si=0.0)

    theta_1 = 0.0
    theta_2 = 0.0
    u = np.array([0.0, 0.0, 4.0, 2.0])  # initial [si1_dot, si2_dot, v1, v2]

    for _ in range(total_steps):
        if u[2] > u[1]:
            B_curr = np.array([
                [0.0, 0.0, 1.0, -1.0],
                [0.0, 0.0, 0.0, 0.0],
                [v1_dash, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, v2_dash, 0.0, 0.0],
            ])
        else:
            B_curr = np.array([
                [0.0, 0.0, -1.0, 1.0],
                [0.0, 0.0, 0.0, 0.0],
                [v1_dash, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, v2_dash, 0.0, 0.0],
            ])

        K = lqr(A, B_curr, Q, R)

        alpha = abs(u[2] - u[3])
        d1 = abs(a1.y - target_y1)
        Vd1 = u[2] * np.sin(a1.si - theta_1)
        d2 = abs(a2.y - target_y2)
        Vd2 = u[3] * np.sin(a2.si - theta_2)

        X = np.array([alpha, d1, Vd1, d2, Vd2])
        u = -K @ X

        u[2] = max(u[2], -4.0)
        u[3] = max(u[3], -4.0)

        a1.update(u[0], u[2], dt)
        a2.update(u[1], u[3], dt)

    fig, ax = plot_agents(
        [a1, a2],
        labels=["Agent 1 (y=2)", "Agent 2 (y=4)"],
        title="Two Agent Line Time Coordination",
        reference_lines=[
            {"y": target_y1},
            {"y": target_y2},
        ],
        equal_aspect=False,
        filename="two_agent_line.png",
    )
    print("Plot saved to two_agent_line.png")


if __name__ == "__main__":
    main()
