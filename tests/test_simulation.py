"""Tests for the simulation engine."""

import numpy as np
import pytest
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.path import CirclePath, LinePath
from cooperative_path_following.simulation import Simulation


class TestSingleCircleSimulation:
    """Test single agent circle following."""

    def test_converges_to_circle(self):
        """Agent should converge to the desired circle radius."""
        radius = 2.0
        total_time = 9.0
        path = CirclePath(radius=radius, total_time=total_time)
        A, B = path.state_matrices()
        Q = np.diag([10.0, 1.0])
        R = np.array([[1.0]])
        K = lqr(A, B, Q, R)

        sim = Simulation(dt=0.01, total_steps=900)
        agent = Agent(x=radius, y=0.0, si=-np.pi / 2)
        sim.add_agent(agent)
        sim.run_single_circle(radius, path.v_dash, K)

        # Check that agent stays near the circle
        final_R = np.sqrt(agent.x ** 2 + agent.y ** 2)
        assert abs(final_R - radius) < 0.5  # within 0.5m of desired radius

    def test_history_length(self):
        """History should have total_steps + 1 entries."""
        radius = 2.0
        path = CirclePath(radius=radius, total_time=9.0)
        A, B = path.state_matrices()
        Q = np.diag([10.0, 1.0])
        R = np.array([[1.0]])
        K = lqr(A, B, Q, R)

        steps = 100
        sim = Simulation(dt=0.01, total_steps=steps)
        agent = Agent(x=radius, y=0.0, si=-np.pi / 2)
        sim.add_agent(agent)
        sim.run_single_circle(radius, path.v_dash, K)

        assert len(agent.x_history) == steps + 1
        assert len(agent.y_history) == steps + 1


class TestThreeAgentCircleSimulation:
    """Test three-agent circle time coordination."""

    def test_runs_without_error(self):
        """Three agent circle simulation should run without exceptions."""
        radius1, radius2, radius3 = 65.0, 80.0, 100.0
        total_time = 10.0
        path1 = CirclePath(radius1, total_time=total_time)
        path2 = CirclePath(radius2, total_time=total_time)
        path3 = CirclePath(radius3, total_time=total_time)

        A1, B1 = path1.agent_matrices_3x2()
        K1 = lqr(A1, B1, np.diag([1000.0, 10.0, 1.0]), np.eye(2))

        A3, B3 = path3.agent_matrices_3x2()
        K3 = lqr(A3, B3, np.diag([1000.0, 1.0, 1.0]), np.eye(2))

        A2, B2 = path2.middle_agent_matrices()
        K2 = lqr(A2, B2, np.diag([100.0, 100.0, 100.0, 1.0]), np.eye(2))

        a1 = Agent(x=0.0, y=radius1, si=0.0)
        a2 = Agent(x=radius2, y=0.0, si=-np.pi / 2)
        a3 = Agent(x=radius3 / np.sqrt(2), y=radius3 / np.sqrt(2),
                    si=-np.pi / 4)

        sim = Simulation(dt=0.01, total_steps=200)
        sim.add_agent(a1)
        sim.add_agent(a2)
        sim.add_agent(a3)

        sim.run_three_agent_circle(
            radii=(radius1, radius2, radius3),
            v_dashes=(path1.v_dash, path2.v_dash, path3.v_dash),
            K1=K1, K2=K2, K3=K3,
        )

        # All agents should have histories
        assert len(a1.x_history) == 201
        assert len(a2.x_history) == 201
        assert len(a3.x_history) == 201


class TestThreeAgentLineSimulation:
    """Test three-agent line time coordination."""

    def test_runs_without_error(self):
        """Three agent line simulation should run without exceptions."""
        v_dash = 20.0
        total_time = 10.0
        path = LinePath(target_y=0.0, total_distance=v_dash * total_time,
                        total_time=total_time)

        A1, B1 = path.agent_matrices_3x2()
        K1 = lqr(A1, B1, np.diag([1000.0, 1.0, 1.0]), np.eye(2))

        A3, B3 = path.agent_matrices_3x2()
        K3 = lqr(A3, B3, np.diag([1000.0, 1.0, 1.0]), np.eye(2))

        A2, B2 = path.middle_agent_matrices()
        K2 = lqr(A2, B2, np.diag([100.0, 100.0, 1.0, 1.0]), np.eye(2))

        r1, r2, r3 = 65.0, 80.0, 100.0
        s = np.sqrt(2)
        a1 = Agent(x=-r1 / s, y=-r1 / s, si=0.0)
        a2 = Agent(x=-r2 / s, y=-r2 / s, si=0.0)
        a3 = Agent(x=-r3 / s, y=-r3 / s, si=0.0)

        sim = Simulation(dt=0.01, total_steps=200)
        sim.add_agent(a1)
        sim.add_agent(a2)
        sim.add_agent(a3)

        sim.run_three_agent_line(
            targets_y=(-r1 / s, -r2 / s, -r3 / s),
            v_dash=v_dash,
            K1=K1, K2=K2, K3=K3,
        )

        assert len(a1.x_history) == 201
        # Agents should have moved forward (increasing x)
        assert a1.x > a1.x_history[0]
