"""
Command-line execution entry point for fieldroute: `python -m fieldroute`
"""

from shapely.geometry import Polygon
from .core import Tractor
from .planner import make_passes, best_angle
from .simulator import static_dock, simulate
from .metrics import calculate_metrics, print_summary
from .visualizer import plot, export_interactive_html


def main():
    field = Polygon([(0, 0), (400, 0), (430, 250), (200, 320), (-20, 260)])
    pond = Polygon([(150, 120), (210, 120), (210, 170), (150, 170)])
    obstacles = [pond]

    tr = Tractor(width=6.0, overlap_ratio=0.03, tank=40.0, min_turn_radius=4.5)

    angle = best_angle(field, obstacles, tr.width, tr.overlap_ratio)
    passes = make_passes(field, obstacles, tr.width, angle, tr.overlap_ratio)

    dock = static_dock(-30, -30)
    res = simulate(passes, tr, dock, obstacles=obstacles)
    metrics = calculate_metrics(field, obstacles, passes, tr, res)

    print_summary(metrics, angle, len(passes))
    plot(field, obstacles, res, dock, metrics, path="fieldroute.png")
    export_interactive_html(field, obstacles, res, dock, metrics, tr, path="fieldroute_dashboard.html")


if __name__ == "__main__":
    main()
