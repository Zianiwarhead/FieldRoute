"""
FieldRoute simulator - runs a full fuel-constrained coverage simulation
with obstacle-aware transit routing, teardrop headland turns, and exact time-series telemetry.
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Callable, List, Tuple, Dict, Any
from shapely.geometry import Polygon, LineString
from .core import Tractor, dist
from .planner import Pass, generate_headland_turn, route_around_obstacles

DockFn = Callable[[float], Tuple[float, float]]


@dataclass
class SimulationResult:
    """Outcome of a full simulation run with complete telemetry timeline."""
    trail: List[Tuple[Tuple[float, float], Tuple[float, float], bool]] = field(default_factory=list)
    segments: List[Dict[str, Any]] = field(default_factory=list)
    refuels: List[Dict[str, Any]] = field(default_factory=list)
    time: float = 0.0
    fuel_used: float = 0.0
    work_m: float = 0.0
    travel_m: float = 0.0

    @property
    def total_m(self) -> float:
        """Total distance travelled (working + travelling)."""
        return self.work_m + self.travel_m

    @property
    def num_refuels(self) -> int:
        """Number of refuel stops performed."""
        return len(self.refuels)

    def __getitem__(self, key: str) -> Any:
        return getattr(self, key)

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)


def static_dock(x: float, y: float) -> DockFn:
    """Create a stationary base station / refuel dock."""
    return lambda t: (x, y)


def moving_dock(waypoints: List[Tuple[float, float]], speed: float) -> DockFn:
    """Create an autonomous mobile fuel tender that travels along waypoints at fixed speed."""
    def at(t: float) -> Tuple[float, float]:
        d = max(t, 0.0) * speed
        for a, b in zip(waypoints, waypoints[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if L == 0.0:
                continue
            if d <= L:
                return (
                    a[0] + (b[0] - a[0]) * d / L,
                    a[1] + (b[1] - a[1]) * d / L,
                )
            d -= L
        return waypoints[-1]

    return at


def path_distance(pts: List[Tuple[float, float]]) -> float:
    """Total length of a multi-point polyline."""
    return sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def simulate(
    passes: List[Pass],
    tr: Tractor,
    dock: DockFn,
    obstacles: List[Polygon] = None,
    debug: bool = False,
) -> SimulationResult:
    """
    Simulate autonomous tractor coverage with dynamic fuel / return-to-dock checks,
    realistic teardrop headland turns, and obstacle avoidance during all deadhead transits.
    """
    if obstacles is None:
        obstacles = []

    pos = dock(0.0)
    t = 0.0
    fuel = tr.tank
    used = 0.0
    delay = 0.0

    trail: List[Tuple[Tuple[float, float], Tuple[float, float], bool]] = []
    segments: List[Dict[str, Any]] = []
    refuels: List[Dict[str, Any]] = []

    def D(tt: float) -> Tuple[float, float]:
        return dock(tt - delay)

    def safe_route(p1: Tuple[float, float], p2: Tuple[float, float]) -> List[Tuple[float, float]]:
        return route_around_obstacles(p1, p2, obstacles, buffer_margin=tr.width / 2.0 + 1.0)

    def meet(p: Tuple[float, float], tt: float) -> Tuple[Tuple[float, float], float, float]:
        """Calculates rendezvous location and arrival time accounting for obstacle routing."""
        arrive = tt
        for _ in range(50):
            dock_target = D(arrive)
            route = safe_route(p, dock_target)
            route_dist = path_distance(route)
            new = tt + route_dist / tr.speed
            done = abs(new - arrive) < 0.05
            arrive = new
            if done:
                break
        q = D(arrive)
        final_route = safe_route(p, q)
        return q, path_distance(final_route), arrive

    def drive_waypoint_path(
        pts: List[Tuple[float, float]],
        seg_type: str,
        status_label: str,
        is_working: bool = False,
        force_warning: bool = False,
    ) -> None:
        nonlocal pos, t, fuel, used
        burn_rate = tr.burn_work if is_working else tr.burn_travel

        for i in range(len(pts) - 1):
            p0, p1 = pts[i], pts[i + 1]
            d = dist(p0, p1)
            if d < 1e-4:
                continue

            dur = d / tr.speed
            burn = d * burn_rate
            t_start = t
            t_end = t + dur
            f_start = fuel
            f_end = max(0.0, fuel - burn)
            is_warn = force_warning or (f_end <= tr.tank * 0.20)

            segments.append({
                "p_start": p0,
                "p_end": p1,
                "t_start": t_start,
                "t_end": t_end,
                "fuel_start": f_start,
                "fuel_end": f_end,
                "seg_type": seg_type,
                "status_label": status_label,
                "is_working": is_working,
                "is_warning": is_warn,
            })
            trail.append((p0, p1, is_working))

            pos = p1
            t = t_end
            fuel = f_end
            used += burn

    def refuel():
        nonlocal t, fuel, delay
        q, d, _ = meet(pos, t)
        if d * tr.burn_travel > fuel:
            raise RuntimeError(
                f"Stranded at {pos}: Cannot reach dock with {fuel:.2f}L remaining. Call support."
            )

        # Drive to dock avoiding obstacles with active low-fuel warning
        dock_path = safe_route(pos, q)
        dock_label = f"LOW FUEL ({fuel:.1f}L / {fuel/tr.tank*100:.0f}%) - TRANSIT TO DOCK"
        drive_waypoint_path(dock_path, "TRANSIT_DOCK", dock_label, is_working=False, force_warning=True)

        refuel_entry = {
            "index": len(refuels) + 1,
            "pos": q,
            "time_s": t,
            "time_h": t / 3600.0,
            "fuel_before": fuel,
            "fuel_added": tr.tank - fuel,
        }
        refuels.append(refuel_entry)

        # Refuel pause segment: tractor is stationary, tank fills up to 100%
        refuel_dur = tr.refuel_time
        f_start = fuel
        f_end = tr.tank
        t_start = t
        t_end = t + refuel_dur

        segments.append({
            "p_start": q,
            "p_end": q,
            "t_start": t_start,
            "t_end": t_end,
            "fuel_start": f_start,
            "fuel_end": f_end,
            "seg_type": "REFUEL",
            "status_label": f"REFUELLING AT DOCK (+{f_end - f_start:.1f} L)",
            "is_working": False,
            "is_warning": False,
        })

        t = t_end
        delay += refuel_dur
        fuel = tr.tank

        if debug:
            print(f"  [Refuel #{refuel_entry['index']}] At dock {q} at t={t_start/3600:.2f}h")

    # Main coverage loop
    for idx, (a, b) in enumerate(passes):
        for _ in range(2):
            to_a_path = safe_route(pos, a)
            go_dist = path_distance(to_a_path)
            work_dist = dist(a, b)
            t_end = t + (go_dist + work_dist) / tr.speed
            _, back_dist, _ = meet(b, t_end)

            need = (
                go_dist * tr.burn_travel
                + work_dist * tr.burn_work
                + back_dist * tr.burn_travel * (1.0 + tr.reserve)
            )

            if need <= fuel:
                break
            refuel()
        else:
            raise RuntimeError("Row length exceeds entire single-tank operational capacity.")

        # Transition to start of row (a)
        if dist(pos, a) > 0.05:
            # Check if this is an adjacent headland turn from previous pass
            if idx > 0 and dist(pos, a) <= tr.width * 2.8 and not any(LineString([pos, a]).intersects(o) for o in obstacles):
                prev_a, prev_b = passes[idx - 1]
                v_fwd = (prev_b[0] - prev_a[0], prev_b[1] - prev_a[1])
                turn_pts = generate_headland_turn(pos, a, v_fwd, min_r=tr.min_turn_radius, num_pts=8)
                drive_waypoint_path(turn_pts, "TURN_HEADLAND", "TEARDROP HEADLAND TURN", is_working=False)
            else:
                transit_pts = safe_route(pos, a)
                drive_waypoint_path(transit_pts, "TRANSIT", "TRANSIT AVOIDING OBSTACLE", is_working=False)

        # Work the row pass (a -> b)
        drive_waypoint_path([a, b], "WORK", f"WORKING ROW ({tr.width:.1f}m IMPLEMENT)", is_working=True)

    # Job complete: return safely to dock
    q, _, _ = meet(pos, t)
    home_path = safe_route(pos, q)
    drive_waypoint_path(home_path, "TRANSIT", "JOB COMPLETE - RETURNING TO DOCK", is_working=False)

    work_d = sum(dist(a, b) for a, b, w in trail if w)
    all_d = sum(dist(a, b) for a, b, w in trail)
    travel_d = all_d - work_d

    return SimulationResult(
        trail=trail,
        segments=segments,
        refuels=refuels,
        time=t,
        fuel_used=used,
        work_m=work_d,
        travel_m=travel_d,
    )
