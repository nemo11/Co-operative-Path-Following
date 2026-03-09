# Co-operative Path Following

A Python implementation of co-operative path following algorithms using LQR
(Linear Quadratic Regulator) control for multi-agent systems.

This is a port of the original MATLAB implementation to Python, using NumPy,
SciPy, and Matplotlib.

## Overview

This project simulates multiple autonomous agents cooperatively following
circular and straight-line paths while maintaining time coordination. The
control algorithm uses LQR (Linear Quadratic Regulator) optimal control to
minimise path following errors and synchronise agents' progress along their
respective paths.

### Key Features

- **Single and multi-agent** path following (1, 2, or 3 agents)
- **Circle and line** path types
- **Time coordination** between agents via coupled LQR controllers
- **Hybrid mode switching** between line and circle paths
- **Modular library** for building custom simulations

## Installation

```bash
pip install -r requirements.txt
```

Or install as a package:

```bash
pip install -e .
```

## Quick Start

```python
import numpy as np
from cooperative_path_following.lqr import lqr
from cooperative_path_following.agent import Agent
from cooperative_path_following.path import CirclePath
from cooperative_path_following.simulation import Simulation
from cooperative_path_following.visualization import plot_agents

# Define a circular path
path = CirclePath(radius=2.0, total_time=9.0)
A, B = path.state_matrices()

# Compute LQR gain
Q = np.diag([10.0, 1.0])
R = np.array([[1.0]])
K = lqr(A, B, Q, R)

# Create and run simulation
sim = Simulation(dt=0.01, total_steps=900)
agent = Agent(x=2.0, y=0.0, si=-np.pi/2)
sim.add_agent(agent)
sim.run_single_circle(radius=2.0, v_dash=path.v_dash, K=K)

# Plot results
plot_agents([agent], title="Circle Following", filename="result.png")
```

## Examples

Run any example from the `examples/` directory:

| Example | Description | MATLAB Original |
|---------|-------------|-----------------|
| `path_following_basic.py` | Basic LQR control demo | `path_following.m` |
| `single_circle.py` | Single agent on a circle | `single_circle_simulation.m` |
| `two_agent_circle.py` | 2 agents, circle, time coordination | `time_coordination_circle_simulation.m` |
| `two_agent_line.py` | 2 agents, line, time coordination | `time_coordination_line_simulation.m` |
| `three_agent_circle.py` | 3 agents, circle, time coordination | `three_agent_circle_timecord.m` |
| `three_agent_line.py` | 3 agents, line, time coordination | `three_agent_line_timecord.m` |
| `line_to_circle.py` | 3 agents, hybrid line→circle | `line_circle.m` |

```bash
cd examples
python single_circle.py
python three_agent_circle.py
python line_to_circle.py
```

## Package Structure

```
cooperative_path_following/
├── __init__.py          # Package exports
├── lqr.py               # LQR controller (continuous-time ARE solver)
├── agent.py             # Agent state management and dynamics
├── path.py              # Path definitions (CirclePath, LinePath)
├── simulation.py        # Simulation engine with Euler integration
└── visualization.py     # Matplotlib plotting utilities

examples/                # Runnable example scripts
tests/                   # Unit tests
```

## Library Modules

### `lqr.py`
Computes the optimal LQR gain matrix `K` by solving the continuous-time
algebraic Riccati equation, equivalent to MATLAB's `lqr(A, B, Q, R)`.

### `agent.py`
The `Agent` class tracks position `(x, y)`, virtual curve parameter `si`,
and provides methods to compute path following errors (radial error for
circles, lateral error for lines) and to update state via Euler integration.

### `path.py`
`CirclePath` and `LinePath` classes define path geometry and build the
state-space matrices `(A, B)` used by the LQR controller. Supports single-agent,
edge-agent (3-state), and middle-agent (4-state) configurations.

### `simulation.py`
The `Simulation` class orchestrates the control loop: computing errors,
applying LQR control, enforcing velocity constraints, and updating agent
states at each time step.

### `visualization.py`
Plotting utilities using Matplotlib to visualise agent trajectories with
reference paths, start/end markers, and labels.

## Algorithm Details

### Control Architecture

Each agent's dynamics are modelled as a linear state-space system:

```
ẋ = Ax + Bu
u = -Kx  (LQR optimal feedback)
```

**Circle path states:** `[coordination_error, radial_error, radial_velocity_error]`

**Line path states:** `[coordination_error, lateral_error, lateral_velocity_error]`

### Time Coordination

For multi-agent systems, edge agents (1 and 3) track the angular/position
difference with their neighbour, while the middle agent (2) coordinates
with both neighbours using a 4-state controller.

### Simulation

State updates use Euler integration with a configurable time step (default
0.01s). Angle wrapping keeps the virtual curve parameter in `[-π, π]`.

## Dependencies

- Python ≥ 3.9
- NumPy ≥ 1.21
- SciPy ≥ 1.7
- Matplotlib ≥ 3.4

## Testing

```bash
pip install pytest
pytest tests/
```
