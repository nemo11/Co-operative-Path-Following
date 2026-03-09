"""Simulation engine for co-operative path following."""

import numpy as np
from cooperative_path_following.agent import Agent, wrap_angle


class Simulation:
    """Run a multi-agent path following simulation.

    Parameters
    ----------
    dt : float
        Integration time step in seconds.
    total_steps : int
        Number of simulation steps.
    """

    def __init__(self, dt=0.01, total_steps=2000):
        self.dt = dt
        self.total_steps = total_steps
        self.agents = []
        self.time = np.arange(total_steps) * dt

    def add_agent(self, agent):
        """Add an agent to the simulation.

        Parameters
        ----------
        agent : Agent
            Agent instance to add.
        """
        self.agents.append(agent)

    def run_single_circle(self, radius, v_dash, K):
        """Run single-agent circle path following.

        Parameters
        ----------
        radius : float
            Circle radius.
        v_dash : float
            Desired linear velocity.
        K : ndarray
            LQR gain matrix (1, 2).
        """
        agent = self.agents[0]
        for _ in range(self.total_steps):
            d, Vd = agent.circle_errors(radius, v_dash)
            X = np.array([d, Vd])
            u = -K @ X
            # Feedforward: subtract nominal angular velocity (v_dash/radius)
            v = u[0] - v_dash / radius
            agent.update(v, v_dash, self.dt)

    def run_three_agent_circle(self, radii, v_dashes, K1, K2, K3):
        """Run 3-agent circle path following with time coordination.

        Parameters
        ----------
        radii : tuple of float
            (radius1, radius2, radius3).
        v_dashes : tuple of float
            (v1_dash, v2_dash, v3_dash).
        K1 : ndarray
            LQR gain for agent 1 (edge), shape (2, 3).
        K2 : ndarray
            LQR gain for agent 2 (middle), shape (2, 4).
        K3 : ndarray
            LQR gain for agent 3 (edge), shape (2, 3).
        """
        a1, a2, a3 = self.agents[0], self.agents[1], self.agents[2]
        r1, r2, r3 = radii
        vd1, vd2, vd3 = v_dashes
        v1 = np.array([0.0, 0.0])
        v2 = np.array([0.0, 0.0])
        v3 = np.array([0.0, 0.0])

        for _ in range(self.total_steps):
            # Compute theta (polar angle)
            theta_1 = a1.theta
            theta_2 = a2.theta
            theta_3 = a3.theta

            # Compute tangent angles
            alpha1 = a1.alpha
            alpha2 = a2.alpha
            alpha3 = a3.alpha

            # Agent 1: coordination with agent 2
            cor1 = theta_1 - theta_2
            d1 = a1.R - r1
            Vd1 = v1[1] * np.sin(a1.si - alpha1)

            # Agent 2: coordination with both agent 1 and 3
            cor2_1 = theta_2 - theta_1
            cor2_2 = theta_2 - theta_3
            d2 = a2.R - r2
            Vd2 = v2[1] * np.sin(a2.si - alpha2)

            # Agent 3: coordination with agent 2
            cor3 = theta_3 - theta_2
            d3 = a3.R - r3
            Vd3 = v3[1] * np.sin(a3.si - alpha3)

            # Build state vectors
            X1 = np.array([cor1, d1, Vd1])
            X2 = np.array([cor2_1, cor2_2, d2, Vd2])
            X3 = np.array([cor3, d3, Vd3])

            # Compute control with feedforward compensation
            # (feedforward = v_dash/2 for angular velocity offset on circles)
            u1 = -K1 @ X1
            v1 = u1 - np.array([0.0, vd1 / 2])

            u2 = -K2 @ X2
            v2 = u2 - np.array([0.0, vd2 / 2])

            u3 = -K3 @ X3
            v3 = u3 - np.array([0.0, vd3 / 2])

            # Velocity constraints: ensure non-negative linear velocity
            # (matches MATLAB: if v(2)<0, v(2)=-v(2); if v2(2)<20, v2(2)=20)
            v1[1] = abs(v1[1])
            v2[1] = max(abs(v2[1]), 20.0)
            v3[1] = abs(v3[1])

            # Update agents
            a2.update(v2[0], v2[1], self.dt)
            a1.update(u1[0], v1[1], self.dt)
            a3.update(u3[0], v3[1], self.dt)

    def run_three_agent_line(self, targets_y, v_dash, K1, K2, K3):
        """Run 3-agent line path following with time coordination.

        Parameters
        ----------
        targets_y : tuple of float
            (target_y1, target_y2, target_y3).
        v_dash : float
            Desired linear velocity.
        K1 : ndarray
            LQR gain for agent 1 (edge), shape (2, 3).
        K2 : ndarray
            LQR gain for agent 2 (middle), shape (2, 4).
        K3 : ndarray
            LQR gain for agent 3 (edge), shape (2, 3).
        """
        a1, a2, a3 = self.agents[0], self.agents[1], self.agents[2]
        ty1, ty2, ty3 = targets_y
        v1 = np.array([0.0, 0.0])
        v2 = np.array([0.0, 0.0])
        v3 = np.array([0.0, 0.0])

        for _ in range(self.total_steps):
            # Agent 1: coordination with agent 2
            cor1 = a1.x - a2.x
            d1 = a1.y - ty1
            Vd1 = v1[1] * np.sin(a1.si)

            # Agent 2: coordination with agents 1 and 3
            cor2_1 = a2.x - a1.x
            cor2_2 = a2.x - a3.x
            d2 = a2.y - ty2
            Vd2 = v2[1] * np.sin(a2.si)

            # Agent 3: coordination with agent 2
            cor3 = a3.x - a2.x
            d3 = a3.y - ty3
            Vd3 = v3[1] * np.sin(a3.si)

            # Build state vectors
            X1 = np.array([cor1, d1, Vd1])
            X2 = np.array([cor2_1, cor2_2, d2, Vd2])
            X3 = np.array([cor3, d3, Vd3])

            # Compute control
            u1 = -K1 @ X1
            v1 = u1.copy()

            u2 = -K2 @ X2
            v2 = u2.copy()

            u3 = -K3 @ X3
            v3 = u3.copy()

            # Velocity constraint for agent 2
            if v2[1] < 20:
                v2[1] = 20

            # Update agents
            a2.update(v2[0], v2[1], self.dt)
            a1.update(u1[0], v1[1], self.dt)
            a3.update(u3[0], v3[1], self.dt)

    def run_hybrid_line_to_circle(self, radii, v_dashes_circle, v_dash_line,
                                  waypoints_line, waypoints_end,
                                  K_line, K_circle,
                                  switch_dist_line=10.5, switch_dist_circle=0.1):
        """Run 3-agent hybrid line-to-circle path following.

        Agents start on a line path and switch to circle following based on
        distance thresholds to waypoints.

        Parameters
        ----------
        radii : tuple of float
            (radius1, radius2, radius3).
        v_dashes_circle : tuple of float
            (v1_circle, v2_circle, v3_circle) for circle mode.
        v_dash_line : float
            Linear velocity for line mode.
        waypoints_line : list of (x, y)
            Line start positions for each agent.
        waypoints_end : list of (x, y)
            End positions that trigger circle mode.
        K_line : tuple
            (K1_line, K2_line, K3_line) gain matrices.
        K_circle : tuple
            (K1_circle, K2_circle, K3_circle) gain matrices.
        switch_dist_line : float
            Distance threshold from start to activate line mode.
        switch_dist_circle : float
            Distance threshold from end to activate circle mode.
        """
        a1, a2, a3 = self.agents[0], self.agents[1], self.agents[2]
        r1, r2, r3 = radii
        vc1, vc2, vc3 = v_dashes_circle
        K1_l, K2_l, K3_l = K_line
        K1_c, K2_c, K3_c = K_circle
        w1_l, w2_l, w3_l = waypoints_line
        w1_e, w2_e, w3_e = waypoints_end

        v1 = np.array([0.0, 0.0])
        v2 = np.array([0.0, 0.0])
        v3 = np.array([0.0, 0.0])
        # flags: 1=circle, 2=line
        flag1 = flag2 = flag3 = 2

        for _ in range(self.total_steps):
            # Distances to waypoints
            dist1_start = np.sqrt((a1.x - w1_l[0]) ** 2 + (a1.y - w1_l[1]) ** 2)
            dist2_start = np.sqrt((a2.x - w2_l[0]) ** 2 + (a2.y - w2_l[1]) ** 2)
            dist3_start = np.sqrt((a3.x - w3_l[0]) ** 2 + (a3.y - w3_l[1]) ** 2)

            dist1_end = np.sqrt((a1.x - w1_e[0]) ** 2 + (a1.y - w1_e[1]) ** 2)
            dist2_end = np.sqrt((a2.x - w2_e[0]) ** 2 + (a2.y - w2_e[1]) ** 2)
            dist3_end = np.sqrt((a3.x - w3_e[0]) ** 2 + (a3.y - w3_e[1]) ** 2)

            # Switch mode based on distance
            if dist1_start <= switch_dist_line:
                flag1 = 2
            if dist2_start <= switch_dist_line:
                flag2 = 2
            if dist3_start <= switch_dist_line:
                flag3 = 2
            if dist1_end <= switch_dist_circle:
                flag1 = 1
            if dist2_end <= switch_dist_circle:
                flag2 = 1
            if dist3_end <= switch_dist_circle:
                flag3 = 1

            # Compute polar angles and tangent angles
            theta_1 = a1.theta
            theta_2 = a2.theta
            theta_3 = a3.theta
            alpha1 = a1.alpha
            alpha2 = a2.alpha
            alpha3 = a3.alpha

            # Compute errors based on mode
            if flag1 == 1:  # circle
                cor1 = theta_1 - theta_2
                cor1 = wrap_angle(cor1)
                d1 = a1.R - r1
                Vd1 = v1[1] * np.sin(a1.si - alpha1)
            else:  # line
                cor1 = a1.x - a2.x
                d1 = a1.y - w1_l[1]
                Vd1 = v1[1] * np.sin(a1.si)

            if flag2 == 1:
                cor2_1 = theta_2 - theta_1
                cor2_1 = wrap_angle(cor2_1)
                cor2_2 = theta_2 - theta_3
                cor2_2 = wrap_angle(cor2_2)
                d2 = a2.R - r2
                Vd2 = v2[1] * np.sin(a2.si - alpha2)
            else:
                cor2_1 = a2.x - a1.x
                cor2_2 = a2.x - a3.x
                d2 = a2.y - w2_l[1]
                Vd2 = v2[1] * np.sin(a2.si)

            if flag3 == 1:
                cor3 = theta_3 - theta_2
                cor3 = wrap_angle(cor3)
                d3 = a3.R - r3
                Vd3 = v3[1] * np.sin(a3.si - alpha3)
            else:
                cor3 = a3.x - a2.x
                d3 = a3.y - w3_l[1]
                Vd3 = v3[1] * np.sin(a3.si)

            # State vectors
            X1 = np.array([cor1, d1, Vd1])
            X2 = np.array([cor2_1, cor2_2, d2, Vd2])
            X3 = np.array([cor3, d3, Vd3])

            # Control based on mode
            if flag1 == 2:
                u1 = -K1_l @ X1
                v1 = u1.copy()
                if v1[1] > -5:
                    v1[1] = -5
            else:
                u1 = -K1_c @ X1
                v1 = u1 - np.array([0.0, vc1 / 2])

            if flag2 == 2:
                u2 = -K2_l @ X2
                v2 = u2.copy()
                if v2[1] > -20:
                    v2[1] = -20
            else:
                u2 = -K2_c @ X2
                v2 = u2 - np.array([0.0, vc2 / 2])

            if flag3 == 2:
                u3 = -K3_l @ X3
                v3 = u3.copy()
                if v3[1] > -18:
                    v3[1] = -18
            else:
                u3 = -K3_c @ X3
                v3 = u3 - np.array([0.0, vc3 / 2])

            # Update agents
            a2.update(u2[0], v2[1], self.dt)
            a1.update(u1[0], v1[1], self.dt)
            a3.update(u3[0], v3[1], self.dt)
