# FieldRoute 🚜

[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![Tests](https://img.shields.io/badge/pytest-12%20passed-success.svg)](tests/)
[![GIS Support](https://img.shields.io/badge/GIS-GeoJSON%20%7C%20WGS84-orange.svg)](fieldroute/geo.py)
[![Autopilot Export](https://img.shields.io/badge/Autopilot-AgOpenGPS%20%7C%20Pixhawk%20%7C%20Nav2-green.svg)](waypoints.csv)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![GitHub Pages Live Demo](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-brightgreen)](https://zianiwarhead.github.io/FieldRoute/)

> **Autonomous Tractor Coverage Path Planning (CPP), GIS Pipeline & Fuel/Battery Return-to-Dock Logistics Simulator.**

FieldRoute is an algorithmic mission planning and operational logistics simulation pipeline designed for autonomous agricultural vehicles (such as electric tractors, autonomous retrofits, and robotic implements). 

It models the entire field operation purely in Python using computational geometry (`Shapely`, `NetworkX`, `NumPy`, `Matplotlib`) with **zero hardware requirements**, while bridging directly into real-world GIS GPS data (`WGS84` GeoJSON) and autopilot waypoints.

---

![FieldRoute Mission Map](fieldroute.png)

---

## 🛰️ System Architecture & Autonomous Vision

FieldRoute operates as the **Global Mission Planner & Energy Logistics Engine** in an autonomous agricultural robotics stack:

```
[ Aerial Drone LiDAR Survey ] (e.g. DJI Zenmuse L3 / RTK Boundary Marker Poles)
               │
               ▼
   [ GIS Field Boundary & Obstacles ] (WGS84 GeoJSON / QGIS / Google Earth)
               │
               ▼
    ┌────────────────────────────────────────────────────────┐
    │                     FIELDROUTE                         │
    │  • Local Geodesic Tangent Projection (WGS84 <-> Meters)│
    │  • Boustrophedon Pass Decomposition                    │
    │  • 5° Orientation Angle Optimizer                      │
    │  • Teardrop / Omega (Ω) Headland Turn Generation       │
    │  • Visibility Graph Dijkstra Obstacle Avoidance        │
    │  • Dynamic Fuel / Battery Return-to-Dock Protocol      │
    └────────────────────────────────────────────────────────┘
               │
               ├──► [ Live Interactive Dashboard ] (zianiwarhead.github.io/FieldRoute)
               ├──► [ Full Mission GeoJSON ] (QGIS / Google Earth)
               └──► [ Autopilot Waypoints CSV ] (Pixhawk / AgOpenGPS / ROS 2 Nav2)
                           │
                           ▼
     [ Tractor Edge Compute: NVIDIA Jetson Thor / Orin / Mini ]
     • Local Perception: 360° Cameras + Thermal (Human/Livestock Safety)
     • Ground LiDAR: Real-time Slope, Soil Compaction & Water-logging
     • Precision Actuation: Variable-rate Seeding & Targeted Herbicide Spraying
```

---

## 🚀 Key Autonomy Features

### 1. Real-World GIS & GPS Projection (`fieldroute/geo.py`)
- Ingests real-world **WGS84 (Latitude, Longitude)** `.geojson` boundary files surveyed via RTK GPS poles or aerial drones.
- Projects GPS coordinates into high-precision local metric Cartesian planes with millimeter accuracy.
- Automatically exports:
  - `mission_plan.geojson`: Visualizable directly in **Google Earth, QGIS, or geojson.io**.
  - `waypoints.csv`: Formatted for standard agricultural autopilots (**AgOpenGPS, Pixhawk Mission Planner, ROS 2 Nav2**).

### 2. Inset Boundary & Obstacle Clearances
- Inward-buffers the field perimeter by half the implement width ($W / 2$) so tools never cross property boundaries or fences.
- Outward-buffers obstacles (ponds, tree groves, rocks) to preserve vehicle safety clearances.

### 3. Optimal Pass Angle Search
- Sweeps candidate row orientations every $5^\circ$ across $0^\circ \dots 180^\circ$.
- Evaluates total path cost (working rows + headland turns + obstacle detours) to select the most energy-efficient driving angle.
- Decomposes into boustrophedon (snake) swaths with an adjustable safety overlap margin (default $3\%$).

### 4. Realistic Teardrop / Omega ($\Omega$) Headland Turns
- Real ag tractors with towed implements cannot pivot on a dime without causing tire scrub and soil compaction.
- When pass spacing is narrower than the tractor's turning diameter ($W < 2 R_{\min}$), FieldRoute automatically generates smooth **Cubic Bézier Teardrop / Bulb Turns** extending into the headland margin.

### 5. Visibility-Graph Obstacle Avoidance Routing
- Whenever transitioning between rows split by an obstacle or driving to/from the base station, FieldRoute computes the shortest collision-free path around the obstacle perimeter using a **visibility graph** and Dijkstra pathfinding.
- **Guaranteed zero transit paths cut across ponds or restricted zones.**

### 6. Dynamic Fuel / Battery Return-to-Dock Protocol
- Before committing to each row pass, FieldRoute evaluates:
  $$\text{Energy Required} = \text{Burn}_{\text{to\_row}} + \text{Burn}_{\text{work\_row}} + \text{Burn}_{\text{to\_dock}} \times (1 + \text{Reserve})$$
- If current fuel or battery state-of-charge (SOC) is insufficient to finish the row and return safely to base, FieldRoute detours to the dock, pauses for a 5-minute automated swap/refill cycle, and resumes from the exact row start.
- Supports both **stationary base stations** and **autonomous mobile tenders** (fuel/battery trucks following perimeter waypoints via a dynamic fixed-point rendezvous solver).

### 7. Interactive Browser Telemetry Dashboard
- Exports an all-in-one HTML5 dashboard ([`fieldroute_dashboard.html`](fieldroute_dashboard.html) / [`index.html`](index.html)) with playback controls ($1\times \dots 60\times$), timeline scrubber, live fuel gauge, active low-fuel warnings, and an animated vehicle position/beacon.

---

## 📊 Mission Summary Output

```text
==========================================================
       FIELDROUTE AUTONOMOUS TRACTOR MISSION SUMMARY      
==========================================================
 Optimal Pass Angle     : 15 deg (73 coverage passes)
----------------------------------------------------------
 FIELD & COVERAGE
  * Total Field Area    : 12.30 ha (123,000 m2)
  * Net Workable Area   : 12.00 ha (120,000 m2)
  * Covered Area        : 11.50 ha (95.8% field coverage)
  * Total Swath Swept   : 118,435 m2
  * Overlap Area        : 3,464 m2 (2.92% overlap)
----------------------------------------------------------
 DISTANCE & EFFICIENCY
  * Working Distance    : 19.74 km
  * Transit / Deadhead  : 4.05 km
  * Total Path Distance : 23.78 km
  * Route Efficiency    : 83.0% (working vs total)
----------------------------------------------------------
 FUEL & LOGISTICS
  * Fuel Consumed       : 87.0 L (7.6 L/ha)
  * Refuel Stops        : 2 trips to dock
----------------------------------------------------------
 MISSION TIME
  * Total Duration      : 2h 48m (10,114 seconds)
    - Working Time      : 2.19 h
    - Driving/Turn Time : 0.45 h
    - Refuelling Pause  : 0.17 h
==========================================================
High-res mission map saved to: fieldroute.png
Interactive telemetry dashboard saved to: fieldroute_dashboard.html
Mission GeoJSON exported to: mission_plan.geojson
Autopilot CSV waypoints (525 points) exported to: waypoints.csv
```

---

## ⚡ Quick Start

### Installation

```bash
git clone https://github.com/Zianiwarhead/FieldRoute.git
cd FieldRoute
pip install -r requirements.txt
```

### Run on Default Benchmark Field

```bash
python fieldroute.py
```

### Run with Real-World GeoJSON / GPS Field

```bash
python fieldroute.py --geojson examples/sample_farm.geojson
```

This will run the mission planner on real GPS coordinates, print the telemetry summary to your terminal, and output:
- `fieldroute.png` (high-res publication map)
- `fieldroute_dashboard.html` (interactive web simulator)
- `mission_plan.geojson` (visualizable in Google Earth / geojson.io)
- `waypoints.csv` (autopilot GPS waypoints)

### Run Unit Tests

```bash
pytest
```
*(All 12 unit tests covering geometry, teardrop turns, obstacle avoidance, dock rendezvous, low-fuel thresholds, and GIS projection roundtrips pass).*

---

## 📁 Repository Structure

```text
├── fieldroute.py             # Root runner & direct script execution
├── index.html                # Live interactive web dashboard for GitHub Pages
├── fieldroute_dashboard.html # Standalone interactive simulator
├── fieldroute.png            # High-resolution HUD map output
├── mission_plan.geojson      # Exported WGS84 trajectory for Google Earth / QGIS
├── waypoints.csv             # Exported GPS autopilot waypoints (AgOpenGPS / Nav2)
├── pyproject.toml            # Packaging & pytest configuration
├── requirements.txt          # Python dependencies
├── README.md                 # Project documentation & mission summary
├── LICENSE                   # MIT License
├── examples/
│   └── sample_farm.geojson   # Real-world 12-hectare farm with pond and solar dock
├── fieldroute/
│   ├── __init__.py           # Package exports
│   ├── __main__.py           # Module CLI entrypoint with argparse
│   ├── core.py               # Tractor dataclass and physics parameters
│   ├── geo.py                # WGS84 geodesic projection, GeoJSON loader & waypoint exporter
│   ├── planner.py            # Pass generation, teardrop turns, visibility graph router
│   ├── simulator.py          # Dynamic return-to-dock logic & time-series telemetry
│   ├── metrics.py            # Coverage %, overlap area, distances & KPIs
│   └── visualizer.py         # Matplotlib rendering & HTML dashboard generator
└── tests/
    └── test_fieldroute.py    # 12 comprehensive pytest test cases
```

---

## 📄 License

MIT License. Copyright (c) 2026 Brian Ziani (@Zianiwarhead).
