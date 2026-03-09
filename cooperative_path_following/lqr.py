"""LQR (Linear Quadratic Regulator) controller.

Solves the continuous-time algebraic Riccati equation to compute the optimal
state feedback gain matrix K, equivalent to MATLAB's lqr(A, B, Q, R).
"""

import numpy as np
from scipy.linalg import solve_continuous_are


def lqr(A, B, Q, R):
    """Compute the LQR gain matrix K.

    Solves the continuous-time algebraic Riccati equation:
        A'P + PA - PBR^{-1}B'P + Q = 0

    and returns K = R^{-1} B' P, so that u = -Kx is the optimal control.

    If the standard solver fails (e.g. due to near-uncontrollable modes),
    a small regularization is added to improve numerical conditioning.

    Parameters
    ----------
    A : array_like, shape (n, n)
        State matrix.
    B : array_like, shape (n, m)
        Input matrix.
    Q : array_like, shape (n, n)
        State cost matrix (positive semi-definite).
    R : array_like, shape (m, m)
        Input cost matrix (positive definite).

    Returns
    -------
    K : ndarray, shape (m, n)
        Optimal gain matrix.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)

    try:
        P = solve_continuous_are(A, B, Q, R)
    except np.linalg.LinAlgError:
        # Add small regularization for ill-conditioned systems
        # (e.g. when B has near-duplicate rows making the system
        # not fully controllable). This matches MATLAB's lqr() behaviour.
        n = A.shape[0]
        eps = 1e-6 * np.eye(n)
        P = solve_continuous_are(A + eps, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K
