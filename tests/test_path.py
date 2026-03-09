"""Tests for path definitions."""

import numpy as np
import pytest
from cooperative_path_following.path import CirclePath, LinePath


class TestCirclePath:
    """Test CirclePath class."""

    def test_v_dash(self):
        path = CirclePath(radius=2.0, total_time=9.0)
        expected = (2 * np.pi * 2.0) / 9.0
        assert path.v_dash == pytest.approx(expected)

    def test_state_matrices_shape(self):
        path = CirclePath(radius=2.0, total_time=9.0)
        A, B = path.state_matrices()
        assert A.shape == (2, 2)
        assert B.shape == (2, 1)

    def test_agent_matrices_3x2_shape(self):
        path = CirclePath(radius=65.0, total_time=10.0)
        A, B = path.agent_matrices_3x2()
        assert A.shape == (3, 3)
        assert B.shape == (3, 2)

    def test_agent_matrices_3x2_values(self):
        radius = 65.0
        total_time = 10.0
        path = CirclePath(radius=radius, total_time=total_time)
        A, B = path.agent_matrices_3x2()
        v = path.v_dash
        # Check B matrix values match MATLAB
        assert B[0, 1] == pytest.approx(1.0 / radius)
        assert B[2, 0] == pytest.approx(v)
        assert B[2, 1] == pytest.approx(-(v * 2) / radius)

    def test_middle_agent_matrices_shape(self):
        path = CirclePath(radius=80.0, total_time=10.0)
        A, B = path.middle_agent_matrices()
        assert A.shape == (4, 4)
        assert B.shape == (4, 2)

    def test_middle_agent_A_matrix(self):
        path = CirclePath(radius=80.0, total_time=10.0)
        A, B = path.middle_agent_matrices()
        # A should have 1 at (2,3) and zeros elsewhere
        expected_A = np.zeros((4, 4))
        expected_A[2, 3] = 1.0
        np.testing.assert_array_equal(A, expected_A)


class TestLinePath:
    """Test LinePath class."""

    def test_v_dash(self):
        path = LinePath(target_y=2.0, total_distance=20.0, total_time=10.0)
        assert path.v_dash == pytest.approx(2.0)

    def test_agent_matrices_3x2_shape(self):
        path = LinePath(target_y=0.0, total_distance=200.0, total_time=10.0)
        A, B = path.agent_matrices_3x2()
        assert A.shape == (3, 3)
        assert B.shape == (3, 2)

    def test_agent_matrices_3x2_values(self):
        path = LinePath(target_y=0.0, total_distance=200.0, total_time=10.0)
        A, B = path.agent_matrices_3x2()
        v = path.v_dash
        assert B[0, 1] == pytest.approx(1.0)
        assert B[2, 0] == pytest.approx(v)
        assert B[2, 1] == pytest.approx(0.0)

    def test_middle_agent_matrices_shape(self):
        path = LinePath(target_y=0.0, total_distance=200.0, total_time=10.0)
        A, B = path.middle_agent_matrices()
        assert A.shape == (4, 4)
        assert B.shape == (4, 2)

    def test_two_agent_matrices(self):
        path = LinePath(target_y=2.0, total_distance=20.0, total_time=9.0)
        A, B = path.two_agent_matrices()
        assert A.shape == (5, 5)
        assert B.shape == (5, 4)
        assert A[1, 2] == pytest.approx(1.0)
        assert A[3, 4] == pytest.approx(1.0)
