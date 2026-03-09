#!/usr/bin/env python3
"""Three agents following line paths with time coordination.

Port of ``three_agent_line_timecord.m``.
"""

import numpy as np
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.path import LinePath
from cooperative_path_following.simulation import Simulation
from cooperative_path_following.visualization import plot_agents


def main():
    total_time = 10.0
    v_dash = 20.0

    # All three agents have the same desired speed
    path1 = LinePath(target_y=0.0, total_distance=v_dash * total_time,
                     total_time=total_time)
    path2 = LinePath(target_y=0.0, total_distance=v_dash * total_time,
                     total_time=total_time)
    path3 = LinePath(target_y=0.0, total_distance=v_dash * total_time,
                     total_time=total_time)

    radius1, radius2, radius3 = 65.0, 80.0, 100.0

    # Agent 1 (edge) — 3 states: [cor, d, Vd]
    A1, B1 = path1.agent_matrices_3x2()
    Q1 = np.diag([1000.0, 1.0, 1.0])
    R1 = np.eye(2)
    K1 = lqr(A1, B1, Q1, R1)

    # Agent 3 (edge) — 3 states: [cor, d, Vd]
    A3, B3 = path3.agent_matrices_3x2()
    Q3 = np.diag([1000.0, 1.0, 1.0])
    R3 = np.eye(2)
    K3 = lqr(A3, B3, Q3, R3)

    # Agent 2 (middle) — 4 states: [cor_1, cor_2, d, Vd]
    A2, B2 = path2.middle_agent_matrices()
    Q2 = np.diag([100.0, 100.0, 1.0, 1.0])
    R2 = np.eye(2)
    K2 = lqr(A2, B2, Q2, R2)

    print(f"K1 shape: {K1.shape}, K2 shape: {K2.shape}, K3 shape: {K3.shape}")

    # Initial positions at bottom-left diagonal
    y1 = -radius1 / np.sqrt(2)
    y2 = -radius2 / np.sqrt(2)
    y3 = -radius3 / np.sqrt(2)
    a1 = Agent(x=-radius1 / np.sqrt(2), y=y1, si=0.0)
    a2 = Agent(x=-radius2 / np.sqrt(2), y=y2, si=0.0)
    a3 = Agent(x=-radius3 / np.sqrt(2), y=y3, si=0.0)

    sim = Simulation(dt=0.01, total_steps=2000)
    sim.add_agent(a1)
    sim.add_agent(a2)
    sim.add_agent(a3)

    sim.run_three_agent_line(
        targets_y=(y1, y2, y3),
        v_dash=v_dash,
        K1=K1, K2=K2, K3=K3,
    )

    fig, ax = plot_agents(
        [a1, a2, a3],
        labels=["Agent 1", "Agent 2", "Agent 3"],
        title="Three Agent Line Time Coordination",
        reference_lines=[
            {"y": y1},
            {"y": y2},
            {"y": y3},
        ],
        equal_aspect=False,
        filename="three_agent_line.png",
    )
    print("Plot saved to three_agent_line.png")


if __name__ == "__main__":
    main()
