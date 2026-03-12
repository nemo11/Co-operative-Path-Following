"""Visualization utilities for path following simulations."""

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


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


def animate_agents(agents, labels=None, title="Co-operative Path Following",
                   colors=None, reference_circles=None, reference_lines=None,
                   equal_aspect=True, filename="animation.gif", interval=20,
                   step=10, fps=30):
    """Create an animation showing the progression of agent motion.

    Parameters
    ----------
    agents : list of Agent
        Agents whose trajectories to animate.
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
    filename : str
        Output filename.  Use ``.gif`` for GIF (requires Pillow) or
        ``.mp4`` for video (requires ffmpeg).
    interval : int
        Delay between frames in milliseconds.
    step : int
        Number of simulation steps per animation frame.  Increase to
        shorten the output or decrease for smoother playback.
    fps : int
        Frames per second in the saved file.

    Returns
    -------
    anim : matplotlib.animation.FuncAnimation
        The animation object.
    """
    default_colors = ["blue", "red", "green", "orange", "purple", "cyan"]
    if colors is None:
        colors = default_colors[: len(agents)]
    if labels is None:
        labels = [f"Agent {i + 1}" for i in range(len(agents))]

    n_points = len(agents[0].x_history)
    frame_indices = list(range(0, n_points, step))
    if frame_indices[-1] != n_points - 1:
        frame_indices.append(n_points - 1)

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

    # Compute axis limits from full trajectories
    all_x = [x for a in agents for x in a.x_history]
    all_y = [y for a in agents for y in a.y_history]
    margin = 0.1 * max(max(all_x) - min(all_x), max(all_y) - min(all_y), 1.0)
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    ax.plot(0, 0, "ko", markersize=4)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    if equal_aspect:
        ax.set_aspect("equal")

    # Create line and marker artists for each agent
    lines = []
    markers = []
    for color, label in zip(colors, labels):
        (line,) = ax.plot([], [], color=color, linewidth=2, label=label)
        (marker,) = ax.plot([], [], "o", color=color, markersize=8)
        lines.append(line)
        markers.append(marker)
    ax.legend()

    def init():
        for line, marker in zip(lines, markers):
            line.set_data([], [])
            marker.set_data([], [])
        return lines + markers

    def update(frame_idx):
        idx = frame_indices[frame_idx]
        for agent, line, marker in zip(agents, lines, markers):
            line.set_data(agent.x_history[: idx + 1],
                          agent.y_history[: idx + 1])
            marker.set_data([agent.x_history[idx]], [agent.y_history[idx]])
        return lines + markers

    anim = FuncAnimation(fig, update, init_func=init,
                         frames=len(frame_indices),
                         interval=interval, blit=True)

    if filename.endswith(".mp4"):
        anim.save(filename, writer="ffmpeg", fps=fps, dpi=100)
    else:
        anim.save(filename, writer="pillow", fps=fps)
    print(f"Animation saved to {filename}")

    return anim
