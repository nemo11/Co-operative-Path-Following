#!/usr/bin/env python3
"""Three agents with hybrid line-to-circle path switching.

Port of ``line_circle.m``.

Agents start on line paths and transition to circular path following
when they reach certain waypoints, demonstrating cooperative hybrid
mode switching.
"""

import numpy as np
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.path import CirclePath, LinePath
from cooperative_path_following.simulation import Simulation
from cooperative_path_following.visualization import plot_agents


def main():
    total_time = 10.0
    radius1, radius2, radius3 = 65.0, 80.0, 95.0

    v1_circle_dash = (2 * np.pi * radius1) / total_time
    v2_circle_dash = (2 * np.pi * radius2) / total_time
    v3_circle_dash = (2 * np.pi * radius3) / total_time

    v_line = 20.0

    # ---- Circle controllers ----
    circle1 = CirclePath(radius1, total_time=total_time)
    circle2 = CirclePath(radius2, total_time=total_time)
    circle3 = CirclePath(radius3, total_time=total_time)

    A1c, B1c = circle1.agent_matrices_3x2()
    K1_circle = lqr(A1c, B1c, np.diag([10000.0, 1.0, 1.0]), np.eye(2))

    A3c, B3c = circle3.agent_matrices_3x2()
    K3_circle = lqr(A3c, B3c, np.diag([10000.0, 1.0, 1.0]), np.eye(2))

    A2c, B2c = circle2.middle_agent_matrices()
    K2_circle = lqr(A2c, B2c, np.diag([1000.0, 10000.0, 1.0, 1.0]), np.eye(2))

    # ---- Line controllers ----
    line1 = LinePath(target_y=0.0, total_distance=v_line * total_time,
                     total_time=total_time)

    A1l, B1l = line1.agent_matrices_3x2()
    K1_line = lqr(A1l, B1l, np.diag([1000.0, 1.0, 1.0]), np.eye(2))

    A3l, B3l = line1.agent_matrices_3x2()
    K3_line = lqr(A3l, B3l, np.diag([1000.0, 1.0, 1.0]), np.eye(2))

    A2l, B2l = line1.middle_agent_matrices()
    K2_line = lqr(A2l, B2l, np.diag([100.0, 100.0, 1.0, 1.0]), np.eye(2))

    # ---- Initial state ----
    s = np.sqrt(2)
    # Line start waypoints
    w1_l = (radius1 / s, -radius1 / s)
    w2_l = (radius2 / s, -radius2 / s)
    w3_l = (radius3 / s, -radius3 / s)
    # Circle transition waypoints (end points for line segment)
    w1_e = (-radius1 / s, -radius1 / s)
    w2_e = (-radius2 / s, -radius2 / s)
    w3_e = (-radius3 / s, -radius3 / s)

    a1 = Agent(x=w1_l[0], y=w1_l[1], si=0.0)
    a2 = Agent(x=w2_l[0], y=w2_l[1], si=0.0)
    a3 = Agent(x=w3_l[0], y=w3_l[1], si=0.0)

    sim = Simulation(dt=0.01, total_steps=1600)
    sim.add_agent(a1)
    sim.add_agent(a2)
    sim.add_agent(a3)

    sim.run_hybrid_line_to_circle(
        radii=(radius1, radius2, radius3),
        v_dashes_circle=(v1_circle_dash, v2_circle_dash, v3_circle_dash),
        v_dash_line=v_line,
        waypoints_line=[w1_l, w2_l, w3_l],
        waypoints_end=[w1_e, w2_e, w3_e],
        K_line=(K1_line, K2_line, K3_line),
        K_circle=(K1_circle, K2_circle, K3_circle),
    )

    fig, ax = plot_agents(
        [a1, a2, a3],
        labels=["Agent 1", "Agent 2", "Agent 3"],
        title="Hybrid Line-to-Circle Path Following",
        reference_circles=[
            {"radius": radius1},
            {"radius": radius2},
            {"radius": radius3},
        ],
        filename="line_to_circle.png",
    )
    print("Plot saved to line_to_circle.png")


if __name__ == "__main__":
    main()
