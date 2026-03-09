"""Visualization utilities for path following simulations."""

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_circle_reference(ax, radius, center_x=0.0, center_y=0.0, **kwargs):
    """Draw a reference circle on the given axes.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        Axes to draw on.
    radius : float
        Circle radius.
    center_x, center_y : float
        Circle centre.
    **kwargs
        Additional keyword arguments for ``ax.plot``.
    """
    theta = np.linspace(0, 2 * np.pi, 200)
    x = center_x + radius * np.cos(theta)
    y = center_y + radius * np.sin(theta)
    kwargs.setdefault("linestyle", "--")
    kwargs.setdefault("alpha", 0.3)
    kwargs.setdefault("color", "gray")
    ax.plot(x, y, **kwargs)


def plot_agents(agents, labels=None, title="Co-operative Path Following",
                colors=None, reference_circles=None, reference_lines=None,
                equal_aspect=True, filename=None):
    """Plot the trajectories of all agents.

    Parameters
    ----------
    agents : list of Agent
        Agents whose trajectories to plot.
    labels : list of str, optional
        Label for each agent.
    title : str
        Plot title.
    colors : list of str, optional
        Colour for each agent's trajectory.
    reference_circles : list of dict, optional
        Each dict has keys ``radius``, ``center_x``, ``center_y``.
    reference_lines : list of dict, optional
        Each dict has keys ``y``, ``x_min``, ``x_max``.
    equal_aspect : bool
        Whether to use equal aspect ratio.
    filename : str, optional
        If provided, save the figure to this file.

    Returns
    -------
    fig : matplotlib.figure.Figure
    ax : matplotlib.axes.Axes
    """
    default_colors = ["blue", "red", "green", "orange", "purple", "cyan"]
    if colors is None:
        colors = default_colors[: len(agents)]
    if labels is None:
        labels = [f"Agent {i + 1}" for i in range(len(agents))]

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    # Draw reference paths
    if reference_circles is not None:
        for circ in reference_circles:
            plot_circle_reference(
                ax,
                circ["radius"],
                circ.get("center_x", 0.0),
                circ.get("center_y", 0.0),
            )

    if reference_lines is not None:
        for line in reference_lines:
            ax.axhline(
                y=line["y"],
                xmin=0, xmax=1,
                linestyle="--", alpha=0.3, color="gray",
            )

    # Plot agent trajectories
    for agent, label, color in zip(agents, labels, colors):
        ax.plot(agent.x_history, agent.y_history, color=color,
                linewidth=2, label=label)
        # Mark start position
        ax.plot(agent.x_history[0], agent.y_history[0], "x",
                color=color, markersize=10)
        # Mark end position
        ax.plot(agent.x_history[-1], agent.y_history[-1], "o",
                color=color, markersize=6)

    # Plot centre
    ax.plot(0, 0, "ko", markersize=4)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    if equal_aspect:
        ax.set_aspect("equal")

    if filename is not None:
        fig.savefig(filename, dpi=150, bbox_inches="tight")

    return fig, ax
