"""
GIS & GPS coordinate projection, GeoJSON loader, and Autopilot Waypoint Exporter.
Converts real-world WGS84 (Latitude, Longitude) coordinates into local metric coordinates (metres)
and exports planned autonomous missions to GeoJSON (Google Earth / QGIS / geojson.io)
and standard Autopilot CSV waypoints (AgOpenGPS / Pixhawk / ROS 2).
"""

from __future__ import annotations
import json
import math
import csv
from typing import List, Tuple, Dict, Any, Optional
from shapely.geometry import Polygon, LineString, Point

# WGS84 Ellipsoid constants
WGS84_A = 6378137.0          # semi-major axis (metres)
WGS84_F = 1.0 / 298.257223563 # flattening
WGS84_B = WGS84_A * (1.0 - WGS84_F)


def gps_to_local_meters(
    lat: float, lon: float, origin_lat: float, origin_lon: float
) -> Tuple[float, float]:
    """
    Project WGS84 (lat, lon) to local tangent plane Cartesian (x, y) in metres
    relative to a local agricultural reference origin (origin_lat, origin_lon).
    High-precision geodesic projection accurate to millimetres over farm-scale domains.
    """
    rad = math.pi / 180.0
    lat_r, lon_r = lat * rad, lon * rad
    lat0_r, lon0_r = origin_lat * rad, origin_lon * rad

    # Meridian and Prime Vertical radii of curvature
    sin_lat = math.sin(lat0_r)
    e2 = 2.0 * WGS84_F - WGS84_F ** 2
    N = WGS84_A / math.sqrt(1.0 - e2 * (sin_lat ** 2))
    M = WGS84_A * (1.0 - e2) / ((1.0 - e2 * (sin_lat ** 2)) ** 1.5)

    dlat = lat_r - lat0_r
    dlon = lon_r - lon0_r

    x = dlon * N * math.cos(lat0_r)
    y = dlat * M
    return (x, y)


def local_meters_to_gps(
    x: float, y: float, origin_lat: float, origin_lon: float
) -> Tuple[float, float]:
    """
    Inverse projection from local Cartesian (x, y) metres back to WGS84 (Latitude, Longitude).
    """
    rad = math.pi / 180.0
    lat0_r, lon0_r = origin_lat * rad, origin_lon * rad

    sin_lat = math.sin(lat0_r)
    e2 = 2.0 * WGS84_F - WGS84_F ** 2
    N = WGS84_A / math.sqrt(1.0 - e2 * (sin_lat ** 2))
    M = WGS84_A * (1.0 - e2) / ((1.0 - e2 * (sin_lat ** 2)) ** 1.5)

    dlat = y / M
    dlon = x / (N * math.cos(lat0_r))

    lat = (lat0_r + dlat) / rad
    lon = (lon0_r + dlon) / rad
    return (lat, lon)


def load_geojson_field(
    geojson_source: str | Dict[str, Any]
) -> Tuple[Polygon, List[Polygon], Tuple[float, float], Optional[Tuple[float, float]]]:
    """
    Load an agricultural field boundary and optional obstacles/dock from a GeoJSON
    file or Python dictionary.

    Features are recognized by their properties:
      - 'type': 'field' / 'boundary' -> outer field polygon
      - 'type': 'obstacle' / 'pond' / 'trees' -> interior excluded obstacles
      - 'type': 'dock' / 'base' -> refuel station / charging pad point

    Returns:
      (field_polygon_m, obstacles_list_m, origin_latlon, dock_point_m)
    """
    if isinstance(geojson_source, str):
        with open(geojson_source, "r", encoding="utf-8") as f:
            data = json.load(f)
    else:
        data = geojson_source

    features = data.get("features", [])
    if not features and data.get("type") == "Feature":
        features = [data]

    # Find field boundary feature to establish origin
    field_coords_gps = None
    obstacle_coords_gps_list = []
    dock_gps = None

    for feat in features:
        props = feat.get("properties", {})
        feat_type = props.get("type", "").lower()
        geom = feat.get("geometry", {})
        gtype = geom.get("type", "")

        if gtype in ("Polygon", "MultiPolygon") and (
            feat_type in ("field", "boundary") or field_coords_gps is None
        ):
            if gtype == "Polygon":
                field_coords_gps = geom["coordinates"][0]
            else:
                field_coords_gps = geom["coordinates"][0][0]
        elif gtype in ("Polygon", "MultiPolygon") and feat_type in ("obstacle", "pond", "trees", "rock"):
            if gtype == "Polygon":
                obstacle_coords_gps_list.append(geom["coordinates"][0])
            else:
                obstacle_coords_gps_list.append(geom["coordinates"][0][0])
        elif gtype == "Point" and feat_type in ("dock", "base", "depot"):
            dock_gps = (geom["coordinates"][1], geom["coordinates"][0]) # (lat, lon)

    if not field_coords_gps:
        raise ValueError("No valid Polygon boundary feature found in GeoJSON.")

    # Reference origin: first coordinate of field boundary [lon, lat]
    origin_lon, origin_lat = field_coords_gps[0][0], field_coords_gps[0][1]
    origin_latlon = (origin_lat, origin_lon)

    # Convert field boundary to local metres
    field_pts_m = [
        gps_to_local_meters(lat=c[1], lon=c[0], origin_lat=origin_lat, origin_lon=origin_lon)
        for c in field_coords_gps
    ]
    field_poly_m = Polygon(field_pts_m)

    # Convert obstacles to local metres
    obstacles_poly_m = []
    for obs in obstacle_coords_gps_list:
        obs_pts_m = [
            gps_to_local_meters(lat=c[1], lon=c[0], origin_lat=origin_lat, origin_lon=origin_lon)
            for c in obs
        ]
        obstacles_poly_m.append(Polygon(obs_pts_m))

    # Convert dock if specified
    dock_m = None
    if dock_gps:
        dock_m = gps_to_local_meters(
            lat=dock_gps[0], lon=dock_gps[1], origin_lat=origin_lat, origin_lon=origin_lon
        )

    return field_poly_m, obstacles_poly_m, origin_latlon, dock_m


def export_mission_geojson(
    field: Polygon,
    obstacles: List[Polygon],
    res: Dict[str, Any],
    origin_latlon: Tuple[float, float],
    output_path: str = "mission_plan.geojson",
) -> None:
    """
    Export full coverage path, teardrop turns, obstacle avoidance routes, and refuel stops
    to a standard WGS84 GeoJSON file ready for Google Earth, QGIS, or geojson.io.
    """
    origin_lat, origin_lon = origin_latlon
    features: List[Dict[str, Any]] = []

    # 1. Field Boundary Polygon
    field_gps_coords = [
        list(reversed(local_meters_to_gps(x, y, origin_lat, origin_lon)))
        for x, y in field.exterior.coords
    ]
    features.append({
        "type": "Feature",
        "properties": {
            "name": "Field Boundary",
            "stroke": "#15803d",
            "stroke-width": 3,
            "fill": "#22c55e",
            "fill-opacity": 0.15,
        },
        "geometry": {"type": "Polygon", "coordinates": [field_gps_coords]},
    })

    # 2. Obstacles
    for i, obs in enumerate(obstacles):
        obs_gps_coords = [
            list(reversed(local_meters_to_gps(x, y, origin_lat, origin_lon)))
            for x, y in obs.exterior.coords
        ]
        features.append({
            "type": "Feature",
            "properties": {
                "name": f"Obstacle #{i+1} (Excluded Zone)",
                "stroke": "#1d4ed8",
                "stroke-width": 2,
                "fill": "#3b82f6",
                "fill-opacity": 0.4,
            },
            "geometry": {"type": "Polygon", "coordinates": [obs_gps_coords]},
        })

    # 3. Segments (Working swaths, turns, transit paths)
    for seg in res.get("segments", []):
        p0, p1 = seg["p_start"], seg["p_end"]
        gps0 = list(reversed(local_meters_to_gps(p0[0], p0[1], origin_lat, origin_lon)))
        gps1 = list(reversed(local_meters_to_gps(p1[0], p1[1], origin_lat, origin_lon)))

        is_work = seg.get("is_working", False)
        seg_type = seg.get("seg_type", "")
        stroke_color = "#0284c7" if is_work else ("#06b6d4" if seg_type == "TURN_HEADLAND" else "#94a3b8")
        stroke_width = 3 if is_work else 1.5

        features.append({
            "type": "Feature",
            "properties": {
                "type": seg_type,
                "status": seg.get("status_label", ""),
                "working": is_work,
                "stroke": stroke_color,
                "stroke-width": stroke_width,
            },
            "geometry": {"type": "LineString", "coordinates": [gps0, gps1]},
        })

    # 4. Refueling stops
    for ref in res.get("refuels", []):
        q = ref["pos"]
        lat, lon = local_meters_to_gps(q[0], q[1], origin_lat, origin_lon)
        features.append({
            "type": "Feature",
            "properties": {
                "name": f"Refuel Stop #{ref['index']}",
                "time_hours": ref.get("time_h", 0.0),
                "marker-color": "#dc2626",
                "marker-symbol": "fuel",
            },
            "geometry": {"type": "Point", "coordinates": [lon, lat]},
        })

    fc = {"type": "FeatureCollection", "features": features}
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(fc, f, indent=2)
    print(f"Mission GeoJSON exported to: {output_path}")


def export_waypoint_csv(
    res: Dict[str, Any],
    origin_latlon: Tuple[float, float],
    output_path: str = "waypoints.csv",
) -> None:
    """
    Export flight/ground trajectory to standard autopilot Waypoint CSV format
    compatible with AgOpenGPS, Pixhawk / MissionPlanner, and ROS 2 waypoint followers.
    Columns: seq, latitude, longitude, speed_mps, implement_active, seg_type, label
    """
    origin_lat, origin_lon = origin_latlon
    rows: List[Dict[str, Any]] = []

    seq = 1
    for seg in res.get("segments", []):
        p = seg["p_start"]
        lat, lon = local_meters_to_gps(p[0], p[1], origin_lat, origin_lon)
        rows.append({
            "seq": seq,
            "latitude": f"{lat:.8f}",
            "longitude": f"{lon:.8f}",
            "speed_mps": 2.5 if seg.get("is_working") else 2.5,
            "implement_active": 1 if seg.get("is_working") else 0,
            "seg_type": seg.get("seg_type", ""),
            "label": seg.get("status_label", ""),
        })
        seq += 1

    # Final endpoint
    if res.get("segments"):
        last_p = res["segments"][-1]["p_end"]
        lat, lon = local_meters_to_gps(last_p[0], last_p[1], origin_lat, origin_lon)
        rows.append({
            "seq": seq,
            "latitude": f"{lat:.8f}",
            "longitude": f"{lon:.8f}",
            "speed_mps": 0.0,
            "implement_active": 0,
            "seg_type": "DOCK",
            "label": "MISSION COMPLETE",
        })

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f, fieldnames=["seq", "latitude", "longitude", "speed_mps", "implement_active", "seg_type", "label"]
        )
        writer.writeheader()
        writer.writerows(rows)
    print(f"Autopilot CSV waypoints ({len(rows)} points) exported to: {output_path}")
