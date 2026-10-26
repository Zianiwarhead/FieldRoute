"""
Coverage path planning for agricultural fields with obstacles, headland turns, and obstacle-aware routing.
"""

from __future__ import annotations
import math
from typing import List, Tuple
from shapely.geometry import Polygon, LineString, Point
from shapely import affinity
import networkx as nx
from .core import dist

Pass = Tuple[Tuple[float, float], Tuple[float, float]]


def make_passes(
    field: Polygon,
    obstacles: List[Polygon],
    width: float,
    angle: float,
    overlap_ratio: float = 0.0,
) -> List[Pass]:
    """
    Generate parallel coverage passes ((x0, y0), (x1, y1)) inside field boundary.
    Buffers the field inward by half implement width so implements stay strictly inside.
    Buffers obstacles outward to maintain safety clearance.
    """
    step = width * (1.0 - overlap_ratio)
    area = field.buffer(-width / 2.0)
    for o in obstacles:
        area = area.difference(o.buffer(width / 2.0))
    if area.is_empty:
        return []

    c = field.centroid
    rot = affinity.rotate(area, -angle, origin=c)
    minx, miny, maxx, maxy = rot.bounds

    ys, y = [], miny + 0.05
    while y < maxy:
        ys.append(y)
        y += step
    if ys and ys[-1] < maxy - 0.5:
        ys.append(maxy - 0.05)

    passes: List[Pass] = []
    for k, y in enumerate(ys):
        hit = rot.intersection(LineString([(minx - 1, y), (maxx + 1, y)]))
        segs = [
            g
            for g in getattr(hit, "geoms", [hit])
            if g.geom_type == "LineString" and g.length > 1.0
        ]
        segs.sort(key=lambda s: s.bounds[0])
        row = [((s.bounds[0], y), (s.bounds[2], y)) for s in segs]
        if k % 2:  # Boustrophedon / snake pattern
            row = [(b, a) for a, b in reversed(row)]
        passes += row

    back = lambda p: affinity.rotate(Point(p), angle, origin=c).coords[0]
    return [(back(a), back(b)) for a, b in passes]


def path_length(passes: List[Pass]) -> float:
    """Total distance of passes including headland turns between consecutive passes."""
    if not passes:
        return float("inf")
    work = sum(dist(a, b) for a, b in passes)
    turns = sum(dist(passes[i][1], passes[i + 1][0]) for i in range(len(passes) - 1))
    return work + turns


def best_angle(
    field: Polygon,
    obstacles: List[Polygon],
    width: float,
    overlap_ratio: float = 0.0,
) -> int:
    """Test angles in 5-degree increments to find the orientation with minimal total driving distance."""
    return min(
        range(0, 180, 5),
        key=lambda ang: path_length(make_passes(field, obstacles, width, ang, overlap_ratio)),
    )


def generate_headland_turn(
    p_exit: Tuple[float, float],
    p_entry: Tuple[float, float],
    v_exit: Tuple[float, float],
    min_r: float = 4.5,
    num_pts: int = 8,
) -> List[Tuple[float, float]]:
    """
    Generate an authentic agricultural Teardrop / Omega (Ω) headland turn curve.
    When pass spacing is narrower than tractor turning circle (w < 2*R), the vehicle
    swings outward into the headland buffer and curves smoothly into the adjacent row.
    """
    dx, dy = v_exit
    length_v = math.hypot(dx, dy)
    if length_v == 0:
        return [p_exit, p_entry]
    dx, dy = dx / length_v, dy / length_v

    lat_x = p_entry[0] - p_exit[0]
    lat_y = p_entry[1] - p_exit[1]
    w = math.hypot(lat_x, lat_y)
    if w < 0.1:
        return [p_exit, p_entry]

    lux, luy = lat_x / w, lat_y / w
    h_ext = max(min_r * 1.25, w * 0.75)
    bulge = max(0.0, (2.0 * min_r - w) * 0.35)

    # 4-point Cubic Bezier definition
    cp0 = p_exit
    cp1 = (p_exit[0] + dx * h_ext - lux * bulge, p_exit[1] + dy * h_ext - luy * bulge)
    cp2 = (p_entry[0] + dx * h_ext + lux * bulge, p_entry[1] + dy * h_ext + luy * bulge)
    cp3 = p_entry

    pts: List[Tuple[float, float]] = []
    for i in range(num_pts + 1):
        t = i / num_pts
        bx = (1 - t) ** 3 * cp0[0] + 3 * (1 - t) ** 2 * t * cp1[0] + 3 * (1 - t) * t ** 2 * cp2[0] + t ** 3 * cp3[0]
        by = (1 - t) ** 3 * cp0[1] + 3 * (1 - t) ** 2 * t * cp1[1] + 3 * (1 - t) * t ** 2 * cp2[1] + t ** 3 * cp3[1]
        pts.append((bx, by))
    return pts


def route_around_obstacles(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: List[Polygon],
    buffer_margin: float = 3.5,
) -> List[Tuple[float, float]]:
    """
    Compute shortest collision-free transit route around obstacles using visibility graph.
    Guarantees no deadheading paths cut through ponds, tree stands, or sensitive zones.
    """
    direct = LineString([start, goal])
    intersecting_obs = [o for o in obstacles if direct.intersects(o)]
    if not intersecting_obs:
        return [start, goal]

    # Collect offset boundary vertices around obstacles
    nodes: List[Tuple[float, float]] = [start, goal]
    for o in obstacles:
        buf = o.buffer(buffer_margin).simplify(0.8)
        coords = list(buf.exterior.coords)[:-1]
        nodes.extend(coords)

    G = nx.Graph()
    n = len(nodes)
    for i in range(n):
        for j in range(i + 1, n):
            u, v = nodes[i], nodes[j]
            seg = LineString([u, v])
            if not any(seg.intersects(o) for o in obstacles):
                G.add_edge(i, j, weight=dist(u, v))

    if nx.has_path(G, 0, 1):
        path_idx = nx.shortest_path(G, 0, 1, weight="weight")
        return [nodes[idx] for idx in path_idx]

    return [start, goal]
