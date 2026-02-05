import math
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

"""Straight-line-to-beacon visualization.

This script generates a straight-line drone trajectory from a start LLA point
to a beacon LLA point and plots the drone path and beacon location (top-down).

Filename: straight_line_to_beacon.py
"""


class LLA_Point:
    def __init__(self, lat, lon, alt=0.0):
        self.lat = lat
        self.lon = lon
        self.alt = alt


def lla_to_ecef(lla_point: LLA_Point):
    """Convert LLA (degrees, degrees, meters) to ECEF (meters).

    Uses WGS84 ellipsoid.
    """
    a = 6378137.0
    f = 1 / 298.257223563
    lat_rad = math.radians(lla_point.lat)
    lon_rad = math.radians(lla_point.lon)
    alt = lla_point.alt
    e2 = f * (2 - f)
    n = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
    x = (n + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (n + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (n * (1 - e2) + alt) * math.sin(lat_rad)
    return np.array([x, y, z])


class Beacon:
    def __init__(self, lla: LLA_Point):
        self.lla = lla
        self.ecef = lla_to_ecef(lla)


class Drone:
    def __init__(self, start_lla: LLA_Point, speed_m_s: float = 20.0):
        self.start_lla = start_lla
        self.start_ecef = lla_to_ecef(start_lla)
        self.speed = speed_m_s

    def straight_trajectory(self, goal_ecef, num_points=100):
        """Return a list of ECEF points interpolating straight from start to goal."""
        points = [
            self.start_ecef + (goal_ecef - self.start_ecef) * (i / (num_points - 1))
            for i in range(num_points)
        ]
        return np.array(points)


class Simulation:
    def __init__(self, drone_start: LLA_Point, beacon_lla: LLA_Point):
        self.drone = Drone(drone_start)
        self.beacon = Beacon(beacon_lla)

    def run(self, num_points=200):
        path = self.drone.straight_trajectory(self.beacon.ecef, num_points)
        return path, self.beacon.ecef


def plot_topdown(ecef_path, beacon_ecef, show=True, save_path=None):
    """Plot top-down view (ECEF X vs Y) of drone path and beacon."""
    xs = ecef_path[:, 0]
    ys = ecef_path[:, 1]

    plt.figure(figsize=(8, 8))
    plt.plot(xs, ys, "-o", markersize=3, label="Drone path")
    plt.scatter([beacon_ecef[0]], [beacon_ecef[1]], c="red", s=80, label="Beacon")
    plt.axis("equal")
    plt.xlabel("ECEF X (m)")
    plt.ylabel("ECEF Y (m)")
    plt.title("Drone Trajectory and Beacon (Top-down ECEF)")
    plt.legend()
    if save_path:
        plt.savefig(save_path, dpi=200)
    # avoid calling plt.show() on non-interactive backends (e.g., Agg)
    if show and "Agg" not in matplotlib.get_backend():
        plt.show()


if __name__ == "__main__":
    # Using the original mission coordinates from the repository
    beacon_lla = LLA_Point(42.628356390253714, -83.10454313558542, 400)
    drone_start_lla = LLA_Point(42.63562993734775, -83.12015983135913, 400)

    sim = Simulation(drone_start_lla, beacon_lla)
    path, beacon_pos = sim.run(num_points=200)

    # save plot for non-interactive environments
    plot_topdown(path, beacon_pos, show=False, save_path="beacon_plot.png")
