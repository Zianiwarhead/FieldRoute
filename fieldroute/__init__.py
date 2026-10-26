"""
FieldRoute - Autonomous Tractor Coverage Path Planning & Refuel-Return Simulator.
"""

from .core import Tractor, dist
from .planner import make_passes, path_length, best_angle, Pass, generate_headland_turn, route_around_obstacles
from .simulator import static_dock, moving_dock, simulate, SimulationResult, DockFn
from .metrics import calculate_metrics, print_summary
from .visualizer import plot, export_interactive_html

__version__ = "1.1.0"

__all__ = [
    "Tractor",
    "dist",
    "Pass",
    "make_passes",
    "path_length",
    "best_angle",
    "generate_headland_turn",
    "route_around_obstacles",
    "DockFn",
    "static_dock",
    "moving_dock",
    "simulate",
    "SimulationResult",
    "calculate_metrics",
    "print_summary",
    "plot",
    "export_interactive_html",
]
