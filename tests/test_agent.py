"""Tests for the Agent class."""

import numpy as np
import pytest
from cooperative_path_following.agent import Agent, wrap_angle, compute_tangent_angle


class TestWrapAngle:
    """Test angle wrapping utility."""

    def test_no_wrap_needed(self):
        assert wrap_angle(0.0) == pytest.approx(0.0)
        assert wrap_angle(1.0) == pytest.approx(1.0)
        assert wrap_angle(-1.0) == pytest.approx(-1.0)

    def test_wrap_positive(self):
        assert wrap_angle(2 * np.pi) == pytest.approx(0.0, abs=1e-10)
        assert wrap_angle(np.pi + 0.1) == pytest.approx(-np.pi + 0.1, abs=1e-10)

    def test_wrap_negative(self):
        assert wrap_angle(-2 * np.pi) == pytest.approx(0.0, abs=1e-10)
        assert wrap_angle(-np.pi - 0.1) == pytest.approx(np.pi - 0.1, abs=1e-10)

    def test_pi_boundary(self):
        result = wrap_angle(np.pi)
        assert abs(result) == pytest.approx(np.pi, abs=1e-10)


class TestComputeTangentAngle:
    """Test tangent angle computation."""

    def test_first_quadrant(self):
        theta = np.pi / 4
        alpha = compute_tangent_angle(theta)
        assert alpha == pytest.approx(theta - np.pi / 2)

    def test_third_quadrant(self):
        theta = -3 * np.pi / 4  # in [-pi, -pi/2]
        alpha = compute_tangent_angle(theta)
        assert alpha == pytest.approx(3 * np.pi / 2 + theta)

    def test_at_zero(self):
        alpha = compute_tangent_angle(0.0)
        assert alpha == pytest.approx(-np.pi / 2)


class TestAgent:
    """Test Agent class."""

    def test_initial_state(self):
        agent = Agent(x=2.0, y=0.0, si=0.0)
        assert agent.x == 2.0
        assert agent.y == 0.0
        assert agent.si == pytest.approx(0.0)
        assert len(agent.x_history) == 1
        assert len(agent.y_history) == 1

    def test_R_property(self):
        agent = Agent(x=3.0, y=4.0, si=0.0)
        assert agent.R == pytest.approx(5.0)

    def test_R_with_center(self):
        agent = Agent(x=4.0, y=3.0, si=0.0, center_x=1.0, center_y=-1.0)
        expected = np.sqrt(9.0 + 16.0)
        assert agent.R == pytest.approx(expected)

    def test_theta_property(self):
        agent = Agent(x=1.0, y=1.0, si=0.0)
        assert agent.theta == pytest.approx(np.pi / 4)

    def test_circle_errors(self):
        agent = Agent(x=3.0, y=0.0, si=0.0)
        d, Vd = agent.circle_errors(radius=2.0, velocity=1.0)
        assert d == pytest.approx(1.0)  # 3 - 2

    def test_line_errors(self):
        agent = Agent(x=0.0, y=2.5, si=0.0)
        d, Vd = agent.line_errors(target_y=2.0, velocity=1.0)
        assert d == pytest.approx(0.5)
        assert Vd == pytest.approx(0.0)  # sin(0) = 0

    def test_update(self):
        agent = Agent(x=0.0, y=0.0, si=0.0)
        agent.update(angular_vel=0.0, linear_vel=1.0, dt=1.0)
        assert agent.x == pytest.approx(1.0)  # cos(0) * 1 * 1
        assert agent.y == pytest.approx(0.0)  # sin(0) * 1 * 1
        assert len(agent.x_history) == 2

    def test_update_with_rotation(self):
        agent = Agent(x=0.0, y=0.0, si=np.pi / 2)
        agent.update(angular_vel=0.0, linear_vel=1.0, dt=1.0)
        assert agent.x == pytest.approx(0.0, abs=1e-10)
        assert agent.y == pytest.approx(1.0)

    def test_angle_wrapping_in_update(self):
        agent = Agent(x=0.0, y=0.0, si=np.pi - 0.1)
        agent.update(angular_vel=10.0, linear_vel=0.0, dt=1.0)
        assert -np.pi <= agent.si <= np.pi
