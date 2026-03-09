"""Tests for the LQR controller."""

import numpy as np
import pytest
from cooperative_path_following.lqr import lqr


class TestLQR:
    """Test the LQR gain computation."""

    def test_simple_double_integrator(self):
        """LQR for a simple double-integrator should return a 1x2 gain."""
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        Q = np.eye(2)
        R = np.array([[1.0]])
        K = lqr(A, B, Q, R)
        assert K.shape == (1, 2)
        # Closed-loop eigenvalues should be stable (negative real parts)
        Ac = A - B @ K
        eigenvalues = np.linalg.eigvals(Ac)
        assert all(np.real(eigenvalues) < 0)

    def test_circle_path_matrices(self):
        """LQR for circle path following should produce stable closed-loop."""
        v = 0.1
        r = 10.0
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0, 0.0], [v, -(v ** 2) / r]])
        Q = np.eye(2)
        R = np.eye(2)
        K = lqr(A, B, Q, R)
        assert K.shape == (2, 2)
        Ac = A - B @ K
        eigenvalues = np.linalg.eigvals(Ac)
        assert all(np.real(eigenvalues) < 0)

    def test_five_state_coordination(self):
        """LQR for 5-state time coordination should return 4x5 gain."""
        v1 = 2.0
        v2 = 1.5
        r1 = 4.0
        r2 = 2.0
        A = np.zeros((5, 5))
        A[1, 2] = 1.0
        A[3, 4] = 1.0
        B = np.array([
            [0.0, 0.0, 1.0 / r1, -1.0 / r2],
            [0.0, 0.0, 0.0, 0.0],
            [v1, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, v2, 0.0, 0.0],
        ])
        Q = np.eye(5)
        R = np.eye(4)
        K = lqr(A, B, Q, R)
        assert K.shape == (4, 5)
        Ac = A - B @ K
        eigenvalues = np.linalg.eigvals(Ac)
        assert all(np.real(eigenvalues) < 0)

    def test_gain_values_double_integrator(self):
        """Known LQR gains for double integrator with identity Q,R."""
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        Q = np.eye(2)
        R = np.array([[1.0]])
        K = lqr(A, B, Q, R)
        # Known analytical solution: K = [1, sqrt(3)]
        np.testing.assert_allclose(K[0, 0], 1.0, atol=1e-10)
        np.testing.assert_allclose(K[0, 1], np.sqrt(3), atol=1e-10)
