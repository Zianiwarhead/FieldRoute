"""
Operational metrics, coverage, and overlap statistics calculations.
"""

from __future__ import annotations
from typing import List, Tuple, Dict, Any
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from .core import Tractor, dist
from .planner import Pass


def calculate_metrics(
    field: Polygon,
    obstacles: List[Polygon],
    passes: List[Pass],
    tr: Tractor,
    res: Any,
) -> Dict[str, Any]:
    """Calculate agricultural area coverage, overlap percentage, distances, and operational KPIs."""
    net_field = field
    for o in obstacles:
        net_field = net_field.difference(o)

    swaths = [LineString([a, b]).buffer(tr.width / 2.0, cap_style="flat") for a, b in passes]
    union_swaths = unary_union(swaths) if swaths else Polygon()
    covered = union_swaths.intersection(net_field) if not union_swaths.is_empty else Polygon()

    total_swath_area = sum(dist(a, b) * tr.width for a, b in passes)
    covered_area = covered.area
    overlap_area = max(0.0, total_swath_area - covered_area)
    overlap_pct = (overlap_area / total_swath_area * 100.0) if total_swath_area > 0 else 0.0
    coverage_pct = (covered_area / net_field.area * 100.0) if net_field.area > 0 else 0.0

    work_m = res["work_m"]
    travel_m = res["travel_m"]
    total_m = work_m + travel_m
    refuels_count = len(res["refuels"])
    mission_time = res["time"]
    refuel_downtime = refuels_count * tr.refuel_time

    return {
        "field_area_m2": field.area,
        "field_area_ha": field.area / 10000.0,
        "net_workable_m2": net_field.area,
        "net_workable_ha": net_field.area / 10000.0,
        "covered_area_m2": covered_area,
        "covered_area_ha": covered_area / 10000.0,
        "coverage_pct": coverage_pct,
        "swath_area_m2": total_swath_area,
        "overlap_area_m2": overlap_area,
        "overlap_pct": overlap_pct,
        "total_distance_m": total_m,
        "total_distance_km": total_m / 1000.0,
        "work_distance_m": work_m,
        "work_distance_km": work_m / 1000.0,
        "travel_distance_m": travel_m,
        "travel_distance_km": travel_m / 1000.0,
        "efficiency_pct": (work_m / total_m * 100.0) if total_m > 0 else 0.0,
        "fuel_used_l": res["fuel_used"],
        "fuel_per_ha": (res["fuel_used"] / (covered_area / 10000.0)) if covered_area > 0 else 0.0,
        "refuels_count": refuels_count,
        "mission_time_s": mission_time,
        "mission_time_h": mission_time / 3600.0,
        "working_time_h": (work_m / tr.speed) / 3600.0,
        "travel_time_h": (travel_m / tr.speed) / 3600.0,
        "refuel_time_h": refuel_downtime / 3600.0,
    }


def print_summary(metrics: Dict[str, Any], angle: int, pass_count: int):
    """Print clean executive summary table to terminal."""
    w = 58
    print("=" * w)
    print("       FIELDROUTE AUTONOMOUS TRACTOR MISSION SUMMARY      ")
    print("=" * w)
    print(f" Optimal Pass Angle     : {angle} deg ({pass_count} coverage passes)")
    print("-" * w)
    print(" FIELD & COVERAGE")
    print(f"  * Total Field Area    : {metrics['field_area_ha']:.2f} ha ({metrics['field_area_m2']:,.0f} m2)")
    print(f"  * Net Workable Area   : {metrics['net_workable_ha']:.2f} ha ({metrics['net_workable_m2']:,.0f} m2)")
    print(f"  * Covered Area        : {metrics['covered_area_ha']:.2f} ha ({metrics['coverage_pct']:.1f}% field coverage)")
    print(f"  * Total Swath Swept   : {metrics['swath_area_m2']:,.0f} m2")
    print(f"  * Overlap Area        : {metrics['overlap_area_m2']:,.0f} m2 ({metrics['overlap_pct']:.2f}% overlap)")
    print("-" * w)
    print(" DISTANCE & EFFICIENCY")
    print(f"  * Working Distance    : {metrics['work_distance_km']:.2f} km")
    print(f"  * Transit / Deadhead  : {metrics['travel_distance_km']:.2f} km")
    print(f"  * Total Path Distance : {metrics['total_distance_km']:.2f} km")
    print(f"  * Route Efficiency    : {metrics['efficiency_pct']:.1f}% (working vs total)")
    print("-" * w)
    print(" FUEL & LOGISTICS")
    print(f"  * Fuel Consumed       : {metrics['fuel_used_l']:.1f} L ({metrics['fuel_per_ha']:.1f} L/ha)")
    print(f"  * Refuel Stops        : {metrics['refuels_count']} trips to dock")
    print("-" * w)
    print(" MISSION TIME")
    hrs = int(metrics['mission_time_h'])
    mins = int((metrics['mission_time_h'] - hrs) * 60)
    print(f"  * Total Duration      : {hrs}h {mins:02d}m ({metrics['mission_time_s']:,.0f} seconds)")
    print(f"    - Working Time      : {metrics['working_time_h']:.2f} h")
    print(f"    - Driving/Turn Time : {metrics['travel_time_h']:.2f} h")
    print(f"    - Refuelling Pause  : {metrics['refuel_time_h']:.2f} h")
    print("=" * w)
