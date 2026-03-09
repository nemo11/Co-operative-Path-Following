"""Path definitions for circle and line paths."""

import numpy as np


class CirclePath:
    """Circular path definition.

    Parameters
    ----------
    radius : float
        Circle radius.
    center_x : float
        Circle centre x-coordinate.
    center_y : float
        Circle centre y-coordinate.
    total_time : float
        Desired time to complete one full revolution.
    """

    def __init__(self, radius, center_x=0.0, center_y=0.0, total_time=10.0):
        self.radius = float(radius)
        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.total_time = float(total_time)

    @property
    def v_dash(self):
        """Desired linear velocity for one revolution in total_time."""
        return (2 * np.pi * self.radius) / self.total_time

    def state_matrices(self):
        """Build A, B matrices for single-agent circle path following.

        State: [d, V_d]
        Input: [u] (angular velocity)

        Returns
        -------
        A : ndarray (2, 2)
        B : ndarray (2, 1)
        """
        A = np.array([[0.0, 1.0],
                       [0.0, 0.0]])
        B = np.array([[0.0],
                       [self.v_dash]])
        return A, B

    def agent_matrices_3x2(self):
        """Build A, B matrices for a 3-state agent with time coordination.

        State: [coordination_error, d, V_d]
        Input: [si_dot, v_linear]

        Returns
        -------
        A : ndarray (3, 3)
        B : ndarray (3, 2)
        """
        v = self.v_dash
        r = self.radius
        A = np.array([[0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [0.0, 0.0, 0.0]])
        B = np.array([[0.0, 1.0 / r],
                       [0.0, 0.0],
                       [v, -(v * 2) / r]])
        return A, B

    def middle_agent_matrices(self):
        """Build A, B matrices for a middle agent (4-state, coupling to two neighbours).

        State: [cor_left, cor_right, d, V_d]
        Input: [si_dot, v_linear]

        Returns
        -------
        A : ndarray (4, 4)
        B : ndarray (4, 2)
        """
        v = self.v_dash
        r = self.radius
        A = np.zeros((4, 4))
        A[2, 3] = 1.0
        B = np.array([[0.0, 1.0 / r],
                       [0.0, 1.0 / r],
                       [0.0, 0.0],
                       [v, -(v * 2) / r]])
        return A, B


class LinePath:
    """Straight line path definition.

    Parameters
    ----------
    target_y : float
        Desired y-coordinate of the line path.
    total_distance : float
        Distance to travel along x.
    total_time : float
        Desired time to cover total_distance.
    """

    def __init__(self, target_y, total_distance=20.0, total_time=10.0):
        self.target_y = float(target_y)
        self.total_distance = float(total_distance)
        self.total_time = float(total_time)

    @property
    def v_dash(self):
        """Desired linear velocity."""
        return self.total_distance / self.total_time

    def agent_matrices_3x2(self):
        """Build A, B matrices for a 3-state agent with time coordination.

        State: [coordination_error, d, V_d]
        Input: [si_dot, v_linear]

        Returns
        -------
        A : ndarray (3, 3)
        B : ndarray (3, 2)
        """
        v = self.v_dash
        A = np.array([[0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [0.0, 0.0, 0.0]])
        B = np.array([[0.0, 1.0],
                       [0.0, 0.0],
                       [v, 0.0]])
        return A, B

    def middle_agent_matrices(self):
        """Build A, B matrices for a middle agent (4-state, coupling to two neighbours).

        State: [cor_left, cor_right, d, V_d]
        Input: [si_dot, v_linear]

        Returns
        -------
        A : ndarray (4, 4)
        B : ndarray (4, 2)
        """
        v = self.v_dash
        A = np.zeros((4, 4))
        A[2, 3] = 1.0
        B = np.array([[0.0, 1.0],
                       [0.0, 1.0],
                       [0.0, 0.0],
                       [v, 0.0]])
        return A, B

    def two_agent_matrices(self):
        """Build A, B matrices for 2-agent line coordination.

        State: [gamma, d1, Vd1, d2, Vd2]
        Input: [u1, u2, u3, u4] (si_dot_1, si_dot_2, v1, v2)

        Returns
        -------
        A : ndarray (5, 5)
        B : ndarray (5, 4)
        """
        v1 = self.v_dash
        v2 = self.v_dash
        A = np.zeros((5, 5))
        A[1, 2] = 1.0
        A[3, 4] = 1.0
        B = np.array([[0.0, 0.0, 1.0, -1.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [v1, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, v2, 0.0, 0.0]])
        return A, B
