"""
Unit tests for FieldRoute coverage path planning, fuel simulation,
teardrop headland turns, obstacle avoidance routing, and GIS/GeoJSON pipelines.
"""

import os
import pytest
from shapely.geometry import Polygon, LineString
from fieldroute import (
    Tractor,
    make_passes,
    best_angle,
    generate_headland_turn,
    route_around_obstacles,
    static_dock,
    moving_dock,
    simulate,
    calculate_metrics,
    gps_to_local_meters,
    local_meters_to_gps,
    load_geojson_field,
    export_mission_geojson,
    export_waypoint_csv,
)


@pytest.fixture
def test_field():
    return Polygon([(0, 0), (200, 0), (200, 100), (0, 100)])


@pytest.fixture
def test_obstacle():
    return Polygon([(50, 40), (90, 40), (90, 60), (50, 60)])


def test_tractor_defaults():
    tr = Tractor()
    assert tr.width == 6.0
    assert tr.speed == 2.5
    assert tr.tank == 40.0
    assert tr.burn_work_km == 4.0
    assert tr.burn_travel_km == 2.0
    assert tr.min_turn_radius == 4.5


def test_make_passes_generation(test_field):
    passes = make_passes(test_field, [], width=10.0, angle=0.0)
    assert len(passes) > 0
    assert 8 <= len(passes) <= 11


def test_obstacle_exclusion(test_field, test_obstacle):
    passes_no_obs = make_passes(test_field, [], width=6.0, angle=0.0)
    passes_with_obs = make_passes(test_field, [test_obstacle], width=6.0, angle=0.0)
    assert len(passes_with_obs) > len(passes_no_obs)


def test_best_angle_finder(test_field):
    ang = best_angle(test_field, [], width=10.0)
    assert 0 <= ang < 180


def test_teardrop_headland_turn():
    p_exit = (100.0, 100.0)
    p_entry = (106.0, 100.0)
    v_exit = (0.0, 1.0)
    turn_pts = generate_headland_turn(p_exit, p_entry, v_exit, min_r=4.5)

    assert len(turn_pts) >= 8
    assert turn_pts[0] == p_exit
    assert turn_pts[-1] == p_entry
    max_y = max(p[1] for p in turn_pts)
    assert max_y > 100.0


def test_route_around_obstacles(test_obstacle):
    start = (20.0, 50.0)
    goal = (120.0, 50.0)
    direct = LineString([start, goal])
    assert direct.intersects(test_obstacle)

    safe_path = route_around_obstacles(start, goal, [test_obstacle], buffer_margin=3.0)
    safe_line = LineString(safe_path)
    assert not safe_line.intersects(test_obstacle)


def test_simulate_static_dock_with_obstacles(test_field, test_obstacle):
    tr = Tractor(width=10.0, tank=50.0)
    passes = make_passes(test_field, [test_obstacle], tr.width, 0.0)
    dock = static_dock(-10.0, -10.0)
    res = simulate(passes, tr, dock, obstacles=[test_obstacle])

    assert res.fuel_used > 0
    assert res.work_m > 0
    assert res.travel_m > 0
    assert res.time > 0
    assert len(res.segments) > 0

    for seg in res.segments:
        line = LineString([seg["p_start"], seg["p_end"]])
        assert not line.intersects(test_obstacle)


def test_simulate_moving_dock(test_field):
    tr = Tractor(width=10.0, tank=50.0)
    passes = make_passes(test_field, [], tr.width, 0.0)
    dock = moving_dock([(-10.0, -10.0), (250.0, -10.0)], speed=0.5)
    res = simulate(passes, tr, dock)

    assert res.fuel_used > 0
    assert res.time > 0


def test_refuel_triggers_on_small_tank(test_field):
    tr = Tractor(width=10.0, tank=2.0)
    passes = make_passes(test_field, [], tr.width, 0.0)
    dock = static_dock(0.0, 0.0)
    res = simulate(passes, tr, dock)

    assert len(res.refuels) > 0


def test_metrics_calculation(test_field):
    tr = Tractor(width=10.0, tank=50.0, overlap_ratio=0.05)
    passes = make_passes(test_field, [], tr.width, 0.0, tr.overlap_ratio)
    dock = static_dock(0.0, 0.0)
    res = simulate(passes, tr, dock)
    metrics = calculate_metrics(test_field, [], passes, tr, res)

    assert "coverage_pct" in metrics
    assert "overlap_pct" in metrics
    assert "fuel_used_l" in metrics
    assert metrics["coverage_pct"] > 80.0
    assert metrics["overlap_pct"] > 0.0


def test_gps_projection_roundtrip():
    origin_lat, origin_lon = -1.292066, 36.821946
    test_lat, test_lon = -1.294500, 36.824500

    x, y = gps_to_local_meters(test_lat, test_lon, origin_lat, origin_lon)
    lat_back, lon_back = local_meters_to_gps(x, y, origin_lat, origin_lon)

    assert abs(lat_back - test_lat) < 1e-7
    assert abs(lon_back - test_lon) < 1e-7


def test_geojson_pipeline(tmp_path):
    geojson_path = "examples/sample_farm.geojson"
    assert os.path.exists(geojson_path)

    field, obstacles, origin, dock = load_geojson_field(geojson_path)
    assert field.area > 50000.0  # > 5 hectares
    assert len(obstacles) == 1
    assert dock is not None

    tr = Tractor(width=8.0, tank=50.0)
    passes = make_passes(field, obstacles, tr.width, 0.0)
    dock_fn = static_dock(*dock)
    res = simulate(passes, tr, dock_fn, obstacles=obstacles)

    out_geojson = str(tmp_path / "test_mission.geojson")
    out_csv = str(tmp_path / "test_waypoints.csv")
    export_mission_geojson(field, obstacles, res, origin, out_geojson)
    export_waypoint_csv(res, origin, out_csv)

    assert os.path.exists(out_geojson)
    assert os.path.exists(out_csv)
    assert os.path.getsize(out_geojson) > 1000
    assert os.path.getsize(out_csv) > 500
