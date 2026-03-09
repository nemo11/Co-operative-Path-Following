"""Co-operative Path Following - Python implementation.

A Python port of the MATLAB co-operative path following simulations using
LQR (Linear Quadratic Regulator) control for multi-agent systems following
circle and line paths with time coordination.
"""

from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.path import CirclePath, LinePath
from cooperative_path_following.simulation import Simulation

__all__ = ["lqr", "Agent", "CirclePath", "LinePath", "Simulation"]
