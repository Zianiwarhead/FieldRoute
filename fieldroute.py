"""
FieldRoute - Autonomous Tractor Coverage Path Planning & Refuel-Return Simulator.
Everything is in metres, litres (or kWh), and seconds.
Run:  python fieldroute.py
"""

from shapely.geometry import Polygon
from fieldroute.core import Tractor, dist
from fieldroute.planner import make_passes, path_length, best_angle, Pass, generate_headland_turn, route_around_obstacles
from fieldroute.simulator import static_dock, moving_dock, simulate, SimulationResult, DockFn
from fieldroute.metrics import calculate_metrics, print_summary
from fieldroute.visualizer import plot, export_interactive_html

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


if __name__ == "__main__":
    # Define test field geometry (metres)
    field = Polygon([(0, 0), (400, 0), (430, 250), (200, 320), (-20, 260)])
    pond = Polygon([(150, 120), (210, 120), (210, 170), (150, 170)])
    obstacles = [pond]

    # Tractor specifications (6m implement, 40L tank, 3% overlap safety margin, 4.5m turn radius)
    tr = Tractor(width=6.0, overlap_ratio=0.03, tank=40.0, min_turn_radius=4.5)

    # Step 1: Find best swath orientation and generate passes
    angle = best_angle(field, obstacles, tr.width, tr.overlap_ratio)
    passes = make_passes(field, obstacles, tr.width, angle, tr.overlap_ratio)

    # Step 2: Choose dock (static or autonomous moving tender)
    dock = static_dock(-30, -30)
    # dock = moving_dock([(-30, -30), (450, -30), (450, 300)], speed=0.5)

    # Step 3: Run simulation with fuel reserve, teardrop turns & obstacle avoidance
    res = simulate(passes, tr, dock, obstacles=obstacles)

    # Step 4: Compute full operational metrics
    metrics = calculate_metrics(field, obstacles, passes, tr, res)

    # Step 5: Report & visualize
    print_summary(metrics, angle, len(passes))
    plot(field, obstacles, res, dock, metrics, path="fieldroute.png")
    export_interactive_html(field, obstacles, res, dock, metrics, tr, path="fieldroute_dashboard.html")
