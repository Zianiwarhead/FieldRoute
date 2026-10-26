"""
Core data structures and math primitives for FieldRoute.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple


def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Euclidean distance between two 2D points (metres)."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class Tractor:
    """
    Tractor implement specifications, steering limits, and operational fuel / energy parameters.
    """
    width: float = 6.0          # implement swath width (m)
    overlap_ratio: float = 0.03 # pass-to-pass overlap safety margin (e.g. 0.03 = 3%)
    speed: float = 2.5          # driving speed m/s (~9 km/h)
    tank: float = 40.0          # litres (or kWh for electric)
    burn_work: float = 0.004    # fuel burn while working (L/m -> 4.0 L/km)
    burn_travel: float = 0.002  # fuel burn while deadheading (L/m -> 2.0 L/km)
    reserve: float = 0.15       # fuel safety reserve buffer (15% for return trip)
    refuel_time: float = 300.0  # seconds per refuel stop (5 minutes)
    min_turn_radius: float = 4.5 # minimum turning radius in metres for headland teardrop turn

    @property
    def burn_work_km(self) -> float:
        """Fuel burn in litres per kilometre while working."""
        return self.burn_work * 1000.0

    @property
    def burn_travel_km(self) -> float:
        """Fuel burn in litres per kilometre while travelling."""
        return self.burn_travel * 1000.0
