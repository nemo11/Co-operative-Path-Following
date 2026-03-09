#!/usr/bin/env python3
"""Two agents following circular paths with time coordination.

Port of ``time_coordination_circle_simulation.m``.
"""

import numpy as np
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.visualization import plot_agents


def main():
    total_time = 9.0
    radius1 = 4.0
    radius2 = 2.0
    v1_dash = (2 * np.pi * radius1) / total_time
    v2_dash = (2 * np.pi * radius2) / total_time

    # State-space model: [gamma, d1, Vd1, d2, Vd2]
    A = np.zeros((5, 5))
    A[1, 2] = 1.0
    A[3, 4] = 1.0

    B = np.array([
        [0.0, 0.0, 1.0 / radius1, -1.0 / radius2, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [v1_dash, 0.0, 0.0, 0.0, -(v1_dash ** 2) / radius1],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, v2_dash, 0.0, 0.0, -(v2_dash ** 2) / radius2],
    ])

    Q = np.eye(5)
    R = np.eye(4)
    K = lqr(A, B[:, :4], Q, R)

    print(f"LQR gain K shape: {K.shape}")

    dt = 0.01
    total_steps = int(total_time / dt)

    # Initial conditions
    a1 = Agent(x=0.0, y=2.0, si=0.0)
    a2 = Agent(x=4.0, y=0.0, si=0.0)

    u = np.array([0.0, 0.0, 4.0, 2.0])  # initial [si1_dot, si2_dot, v1, v2]

    for _ in range(total_steps):
        # Recompute B based on velocity ordering
        if u[2] > u[1]:
            B_curr = np.array([
                [0.0, 0.0, 1.0 / radius1, -1.0 / radius2, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [v1_dash, 0.0, 0.0, 0.0, -(v1_dash ** 2) / radius1],
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, v2_dash, 0.0, 0.0, -(v2_dash ** 2) / radius2],
            ])
        else:
            B_curr = np.array([
                [0.0, 0.0, 1.0 / radius1, -1.0 / radius2, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [v1_dash, 0.0, 0.0, 0.0, -(v1_dash ** 2) / radius1],
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, v2_dash, 0.0, 0.0, -(v2_dash ** 2) / radius2],
            ])

        K = lqr(A, B_curr[:, :4], Q, R)

        R1 = np.sqrt(a1.x ** 2 + a1.y ** 2)
        R2 = np.sqrt(a2.x ** 2 + a2.y ** 2)

        theta_1 = (np.pi / 2) - np.arctan2(a1.y, a1.x + 1e-12)
        theta_2 = (np.pi / 2) - np.arctan2(a2.y, a2.x + 1e-12)

        alpha = abs(theta_1 - theta_2)
        d1 = R1 - radius1
        Vd1 = u[2] * np.sin(a1.si - theta_1)
        d2 = R2 - radius2
        Vd2 = u[3] * np.sin(a2.si - theta_2)

        X = np.array([alpha, d1, Vd1, d2, Vd2])
        u = -K @ X
        u = np.append(u, 1.0)  # feedforward

        u[2] = max(u[2], -4.0)
        u[3] = max(u[3], -4.0)

        # Update agents
        a1.update(u[0], u[2], dt)
        a2.update(u[1], u[3], dt)

    fig, ax = plot_agents(
        [a1, a2],
        labels=["Agent 1 (R=4)", "Agent 2 (R=2)"],
        title="Two Agent Circle Time Coordination",
        reference_circles=[
            {"radius": radius1},
            {"radius": radius2},
        ],
        filename="two_agent_circle.png",
    )
    print("Plot saved to two_agent_circle.png")


if __name__ == "__main__":
    main()
