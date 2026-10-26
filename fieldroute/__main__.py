"""
Command-line execution entry point for fieldroute: `python -m fieldroute`
Supports both default synthetic fields and real-world GeoJSON/GPS field boundaries.
"""

from __future__ import annotations
import argparse
from shapely.geometry import Polygon
from .core import Tractor
from .planner import make_passes, best_angle
from .simulator import static_dock, simulate
from .metrics import calculate_metrics, print_summary
from .visualizer import plot, export_interactive_html
from .geo import load_geojson_field, export_mission_geojson, export_waypoint_csv


def main():
    parser = argparse.ArgumentParser(
        description="FieldRoute: Autonomous Tractor Coverage Path Planner & Refuel Simulator"
    )
    parser.add_argument(
        "--geojson",
        type=str,
        default=None,
        help="Path to GeoJSON field boundary file with GPS (WGS84) coordinates",
    )
    parser.add_argument(
        "--width", type=float, default=6.0, help="Implement swath width in metres (default: 6.0)"
    )
    parser.add_argument(
        "--tank", type=float, default=40.0, help="Fuel / Battery capacity in Litres or kWh (default: 40.0)"
    )
    parser.add_argument(
        "--out-map", type=str, default="fieldroute.png", help="Path to save high-res mission map"
    )
    parser.add_argument(
        "--out-html", type=str, default="fieldroute_dashboard.html", help="Path to save interactive dashboard"
    )
    parser.add_argument(
        "--export-geojson", type=str, default="mission_plan.geojson", help="Path to export mission GeoJSON"
    )
    parser.add_argument(
        "--export-csv", type=str, default="waypoints.csv", help="Path to export autopilot waypoints CSV"
    )
    args = parser.parse_args()

    origin_latlon = None
    dock_pt = (-30.0, -30.0)

    if args.geojson:
        print(f"Loading GPS field boundary from GeoJSON: {args.geojson}")
        field, obstacles, origin_latlon, custom_dock = load_geojson_field(args.geojson)
        if custom_dock:
            dock_pt = custom_dock
    else:
        # Default benchmark field (metres)
        field = Polygon([(0, 0), (400, 0), (430, 250), (200, 320), (-20, 260)])
        pond = Polygon([(150, 120), (210, 120), (210, 170), (150, 170)])
        obstacles = [pond]
        origin_latlon = (-1.292066, 36.821946)

    tr = Tractor(width=args.width, overlap_ratio=0.03, tank=args.tank, min_turn_radius=4.5)

    angle = best_angle(field, obstacles, tr.width, tr.overlap_ratio)
    passes = make_passes(field, obstacles, tr.width, angle, tr.overlap_ratio)

    dock = static_dock(*dock_pt)
    res = simulate(passes, tr, dock, obstacles=obstacles)
    metrics = calculate_metrics(field, obstacles, passes, tr, res)

    print_summary(metrics, angle, len(passes))
    plot(field, obstacles, res, dock, metrics, path=args.out_map)
    export_interactive_html(field, obstacles, res, dock, metrics, tr, path=args.out_html)

    if origin_latlon:
        export_mission_geojson(field, obstacles, res, origin_latlon, output_path=args.export_geojson)
        export_waypoint_csv(res, origin_latlon, output_path=args.export_csv)


if __name__ == "__main__":
    main()
