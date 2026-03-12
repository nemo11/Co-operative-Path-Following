#!/usr/bin/env python3
"""Generate a video showing three agents cooperatively following circular paths.

This example runs the three-agent circle simulation and produces an animated
video file (GIF by default, MP4 if ffmpeg is available) that shows the
progression of motion of the agents over time.
"""

import numpy as np
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.path import CirclePath
from cooperative_path_following.simulation import Simulation
from cooperative_path_following.visualization import animate_agents


def main():
    total_time = 10.0
    radius1, radius2, radius3 = 65.0, 80.0, 100.0

    path1 = CirclePath(radius1, total_time=total_time)
    path2 = CirclePath(radius2, total_time=total_time)
    path3 = CirclePath(radius3, total_time=total_time)

    # Agent 1 (edge) — 3 states: [cor, d, Vd]
    A1, B1 = path1.agent_matrices_3x2()
    K1 = lqr(A1, B1, np.diag([1000.0, 10.0, 1.0]), np.eye(2))

    # Agent 3 (edge) — 3 states: [cor, d, Vd]
    A3, B3 = path3.agent_matrices_3x2()
    K3 = lqr(A3, B3, np.diag([1000.0, 1.0, 1.0]), np.eye(2))

    # Agent 2 (middle) — 4 states: [cor_1, cor_2, d, Vd]
    A2, B2 = path2.middle_agent_matrices()
    K2 = lqr(A2, B2, np.diag([100.0, 100.0, 100.0, 1.0]), np.eye(2))

    # Initial positions on respective circles
    a1 = Agent(x=0.0, y=radius1, si=0.0)
    a2 = Agent(x=radius2, y=0.0, si=-np.pi / 2)
    a3 = Agent(
        x=radius3 / np.sqrt(2),
        y=radius3 / np.sqrt(2),
        si=-np.pi / 4,
    )

    sim = Simulation(dt=0.01, total_steps=2000)
    sim.add_agent(a1)
    sim.add_agent(a2)
    sim.add_agent(a3)

    sim.run_three_agent_circle(
        radii=(radius1, radius2, radius3),
        v_dashes=(path1.v_dash, path2.v_dash, path3.v_dash),
        K1=K1, K2=K2, K3=K3,
    )

    # Generate the animation as a GIF
    animate_agents(
        [a1, a2, a3],
        labels=[
            f"Agent 1 (R={radius1})",
            f"Agent 2 (R={radius2})",
            f"Agent 3 (R={radius3})",
        ],
        title="Three Agent Circle — Cooperative Path Following",
        reference_circles=[
            {"radius": radius1},
            {"radius": radius2},
            {"radius": radius3},
        ],
        filename="three_agent_circle.gif",
        step=20,
        fps=30,
    )


if __name__ == "__main__":
    main()
