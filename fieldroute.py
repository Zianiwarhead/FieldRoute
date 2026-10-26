"""
FieldRoute - Autonomous Tractor Coverage Path Planning & Refuel-Return Simulator.
Everything is in metres, litres (or kWh), and seconds.
Run:
  python fieldroute.py
  python fieldroute.py --geojson examples/sample_farm.geojson
"""

from fieldroute.__main__ import main

if __name__ == "__main__":
    main()
