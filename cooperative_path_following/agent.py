"""Agent state management and dynamics for path following."""

import numpy as np


def wrap_angle(angle):
    """Wrap angle to [-pi, pi].

    Parameters
    ----------
    angle : float
        Angle in radians.

    Returns
    -------
    float
        Wrapped angle in [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def compute_tangent_angle(theta):
    """Compute tangent angle alpha from polar angle theta on a circle.

    For a circle centred at the origin, the tangent direction is
    perpendicular to the radial direction.

    Parameters
    ----------
    theta : float
        Polar angle from atan2(y, x).

    Returns
    -------
    float
        Tangent angle alpha.
    """
    if theta < -np.pi / 2 and theta >= -np.pi:
        alpha = 3 * np.pi / 2 + theta
    else:
        alpha = theta - np.pi / 2
    return alpha


class Agent:
    """Represents a single agent following a path.

    Parameters
    ----------
    x : float
        Initial x position.
    y : float
        Initial y position.
    si : float
        Initial virtual curve parameter (heading angle).
    center_x : float
        Path centre x (for circles) or reference x.
    center_y : float
        Path centre y (for circles) or reference y.
    """

    def __init__(self, x, y, si, center_x=0.0, center_y=0.0):
        self.x = float(x)
        self.y = float(y)
        self.si = wrap_angle(float(si))
        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.x_history = [self.x]
        self.y_history = [self.y]

    @property
    def R(self):
        """Distance from centre."""
        return np.sqrt(
            (self.x - self.center_x) ** 2 + (self.y - self.center_y) ** 2
        )

    @property
    def theta(self):
        """Polar angle from centre."""
        return np.arctan2(
            self.y - self.center_y, self.x - self.center_x
        )

    @property
    def alpha(self):
        """Tangent angle on circle path."""
        return compute_tangent_angle(self.theta)

    def circle_errors(self, radius, velocity):
        """Compute state errors for circle path following.

        Parameters
        ----------
        radius : float
            Desired circle radius.
        velocity : float
            Current linear velocity of the agent.

        Returns
        -------
        d : float
            Radial error (R - radius).
        Vd : float
            Radial velocity error.
        """
        d = self.R - radius
        Vd = velocity * np.sin(self.si - self.alpha)
        return d, Vd

    def line_errors(self, target_y, velocity):
        """Compute state errors for line path following.

        Parameters
        ----------
        target_y : float
            Desired y-coordinate of the line.
        velocity : float
            Current linear velocity of the agent.

        Returns
        -------
        d : float
            Lateral error from line.
        Vd : float
            Lateral velocity error.
        """
        d = self.y - target_y
        Vd = velocity * np.sin(self.si)
        return d, Vd

    def update(self, angular_vel, linear_vel, dt):
        """Update agent state using Euler integration.

        Parameters
        ----------
        angular_vel : float
            Angular velocity command (si_dot).
        linear_vel : float
            Linear velocity command.
        dt : float
            Time step.
        """
        self.si = wrap_angle(self.si + angular_vel * dt)
        self.x += linear_vel * np.cos(self.si) * dt
        self.y += linear_vel * np.sin(self.si) * dt
        self.x_history.append(self.x)
        self.y_history.append(self.y)
