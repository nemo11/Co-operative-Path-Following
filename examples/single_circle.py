#!/usr/bin/env python3
"""Single agent following a circular path using LQR control.

Port of ``single_circle_simulation.m``.
"""

import numpy as np
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.path import CirclePath
from cooperative_path_following.simulation import Simulation
from cooperative_path_following.visualization import plot_agents


def main():
    total_time = 9.0
    radius = 2.0

    path = CirclePath(radius=radius, total_time=total_time)
    A, B = path.state_matrices()

    Q = np.diag([10.0, 1.0])
    R = np.array([[1.0]])
    K = lqr(A, B, Q, R)

    print(f"LQR gain K = {K}")

    sim = Simulation(dt=0.01, total_steps=int(total_time / 0.01))
    agent = Agent(x=radius, y=0.0, si=-np.pi / 2)
    sim.add_agent(agent)
    sim.run_single_circle(radius, path.v_dash, K)

    fig, ax = plot_agents(
        [agent],
        labels=["Agent 1"],
        title="Single Agent Circle Path Following",
        reference_circles=[{"radius": radius}],
        filename="single_circle.png",
    )
    print(f"Final position: ({agent.x:.3f}, {agent.y:.3f})")
    print("Plot saved to single_circle.png")


if __name__ == "__main__":
    main()
