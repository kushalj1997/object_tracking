import gtsam
import random
import math
import multiprocessing as mp
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse

"""Project: visualize covariance reduction for beacon localization.

This file runs a simple 2D Kalman filter (beacon assumed static) and creates an
MP4 animation showing the drone approaching the beacon, measurements, and the
state covariance ellipse shrinking over time.
"""


class LLA_Point:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt


def lla_to_ecef(lla_point: LLA_Point):
    """Convert lla to ecef."""
    a = 6378137.0
    f = 1 / 298.257223563
    lat_rad = math.radians(lla_point.lat)
    lon_rad = math.radians(lla_point.lon)
    alt_rad = lla_point.alt
    e2 = f * (2 - f)
    n = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
    x = (n + alt_rad) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (n + alt_rad) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (n * (1 - e2) + alt_rad) * math.sin(lat_rad)
    return x, y, z


class ECEF_Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def to_llapoint(self):
        """Convert ecef to lla."""
        a = 6378137.0
        f = 1 / 298.257223563
        e2 = f * (2 - f)
        lon = math.atan2(self.y, self.x)
        p = math.sqrt(self.x**2 + self.y**2)
        lat = math.atan2(self.z, p * (1 - e2))
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - n
        return LLA_Point(lat, lon, alt)


class State:
    def __init__(self, lat, lon, alt, vx, vy, vz):
        """Initialize the state of the drone with position in lla and velocity in m/s."""
        self.lla_init = LLA_Point(lat, lon, alt)

        # Convert lla to ecef
        self.ecef_init = lla_to_ecef(self.lla_init)

        self.vx = vx
        self.vy = vy
        self.vz = vz

    def update(self, dt):
        """Move the drone according to its velocity."""
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt


class FactorGraph:
    def __init__(self):
        self.graph = gtsam.NonlinearFactorGraph()
        self.radar_measurement_prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.1, 0.1])
        )

    def add_pose(self, robot_position: ECEF_Point):
        """Add a pose to the factor graph."""
        robot_pose = gtsam.Pose3(
            {0, 0, 0}, robot_position
        )  # convert to gtsam pose todo
        prior = gtsam.PriorFactorPose3(
            1, gtsam.Pose3(robot_pose), self.radar_measurement_prior_noise
        )
        self.graph.add(prior)

    def add_measurement(self, measurement: tuple):
        """Add a radar measurement to the factor graph."""
        radar_measurement = gtsam.RadarFactor(
            1,
            2,
            3,
            measurement[0],
            measurement[1],
            measurement[2],
            measurement[3],
            self.radar_measurement_prior_noise,
        )
        self.graph.add(radar_measurement)


class Drone:
    def __init__(self, initial_state):
        self.state = initial_state
        self.drone_takeoff_velocity = 0.5  # m/s
        self.drone_flight_velocity = 20  # m/s
        self.factor_graph = FactorGraph()

    def plan_path(
        self, start_location: ECEF_Point, end_location: ECEF_Point, total_states=100
    ):
        # Plan a path from the current position to the beacon - straight line path for now
        self.states = []
        for i in range(total_states):
            x = (
                start_location.x
                + (end_location.x - start_location.x) * i / total_states
            )
            y = (
                start_location.y
                + (end_location.y - start_location.y) * i / total_states
            )
            z = (
                start_location.z
                + (end_location.z - start_location.z) * i / total_states
            )
            self.states.append(ECEF_Point(x, y, z))

    def fly_to(self):
        angle = random.uniform(0, 2 * math.pi)
        dx = distance * math.cos(angle)
        dy = distance * math.sin(angle)
        self.position = (self.position[0] + dx, self.position[1] + dy)

    def takeoff(self):
        self.state.vz = 10

    # def spiral(self, location, theta):
    #     """ Fly in a spiral pattern around a location with a constant angular velocity. """
    #     # generate the ecef points of the spiral
    #     r =


class Beacon:
    def __init__(self, height):
        self.height = height
        self.position = (random.uniform(-1000, 1000), random.uniform(-1000, 1000))

    def get_signal_strength(self, drone_position):
        distance = math.sqrt(
            (self.position[0] - drone_position[0]) ** 2
            + (self.position[1] - drone_position[1]) ** 2
        )
        return self.height / (distance)

    def get_azel_from_positions(self, drone_position):
        dx = self.position[0] - drone_position[0]
        dy = self.position[1] - drone_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        azimuth = math.atan2(dy, dx)
        elevation = math.atan2(self.height, distance)
        return azimuth, elevation


class Simulation:
    def __init__(self):
        # mission parameters - fly drone from 3022 penn court to thelma g spencer park
        self.beacon_location = LLA_Point(42.628356390253714, -83.10454313558542, 400)
        self.drone_start_location = LLA_Point(
            42.63562993734775, -83.12015983135913, 400
        )

        self.drone = Drone(self.drone_start_location)
        self.beacon = Beacon(self.beacon_location)

    def step(self, dt):
        self.drone.update(dt)

    def generate_measurements(self, num_measurements):
        measurements = []
        for _ in range(num_measurements):
            self.drone.fly(drone.speed)
            signal_strength = beacon.get_signal_strength(drone.position)
            az, el = beacon.get_azel_from_positions(drone.position)
            measurements.append((drone.position, az, el, signal_strength))
            self.drone.factor_graph.add_measurement(
                (drone.position, az, el, signal_strength)
            )
        return measurements


def cov_ellipse_params(P, nsig=2):
    """Return width, height, angle of covariance ellipse for 2x2 P (nsig sigma)."""
    vals, vecs = np.linalg.eigh(P)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    width, height = 2 * nsig * np.sqrt(vals)
    angle = math.degrees(math.atan2(vecs[1, 0], vecs[0, 0]))
    return width, height, angle


def run_kf_video(output_path="beacon_covariance.mp4", num_steps=200):
    # Ground-truth beacon (from LLA in Simulation)
    beacon_lla = LLA_Point(42.628356390253714, -83.10454313558542, 400)
    drone_start_lla = LLA_Point(42.63562993734775, -83.12015983135913, 400)
    beacon_ecef = np.array(lla_to_ecef(beacon_lla))
    drone_start_ecef = np.array(lla_to_ecef(drone_start_lla))

    # Work in 2D (ECEF X,Y)
    beacon_xy = beacon_ecef[:2]
    drone_path = np.array(
        [
            drone_start_ecef[:2]
            + (beacon_xy - drone_start_ecef[:2]) * (i / (num_steps - 1))
            for i in range(num_steps)
        ]
    )

    # Kalman filter: static state x (beacon position), H=I, F=I
    x = beacon_xy + np.array([5000.0, 5000.0])  # poor initial estimate offset
    P = np.diag([1e6, 1e6])  # very high initial covariance
    Q = np.diag([1.0, 1.0])  # small process noise (beacon static)
    R = np.diag([500.0**2, 500.0**2])  # measurement noise variance

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlabel("ECEF X (m)")
    ax.set_ylabel("ECEF Y (m)")
    ax.set_title("Beacon Localization — Covariance Shrinkage")
    ax.axis("equal")

    (drone_line,) = ax.plot(
        [], [], "-o", color="tab:blue", label="Drone path", markersize=3
    )
    meas_scatter = ax.scatter([], [], c="orange", s=30, label="Measurement")
    (est_point,) = ax.plot(
        [], [], "x", color="green", markersize=10, label="KF estimate"
    )
    cov_ellipse = Ellipse(
        (0, 0), 1, 1, edgecolor="green", facecolor="none", lw=2, label="Covariance"
    )
    ax.add_patch(cov_ellipse)
    ax.scatter([beacon_xy[0]], [beacon_xy[1]], c="red", s=80, label="True beacon")
    ax.legend()

    # persistent text for trace(P)
    trace_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    def init():
        drone_line.set_data([], [])
        meas_scatter.set_offsets(np.empty((0, 2)))
        est_point.set_data([], [])
        cov_ellipse.set_width(0)
        cov_ellipse.set_height(0)
        trace_text.set_text("")
        return drone_line, meas_scatter, est_point, cov_ellipse, trace_text

    def update(i):
        nonlocal x, P
        drone_so_far = drone_path[: i + 1]
        # Simulated measurement: direct noisy observation of beacon position
        meas_noise = np.random.multivariate_normal([0, 0], R)
        z = beacon_xy + meas_noise

        # KF predict (identity)
        x_pred = x
        P_pred = P + Q

        # KF update (H = I)
        S = P_pred + R
        K = P_pred @ np.linalg.inv(S)
        x = x_pred + K @ (z - x_pred)
        P = (np.eye(2) - K) @ P_pred

        # Update plot elements
        drone_line.set_data(drone_so_far[:, 0], drone_so_far[:, 1])
        meas_scatter.set_offsets(z.reshape(1, 2))
        est_point.set_data([x[0]], [x[1]])
        w, h, angle = cov_ellipse_params(P, nsig=2)
        cov_ellipse.set_width(w)
        cov_ellipse.set_height(h)
        cov_ellipse.set_angle(angle)
        cov_ellipse.set_center((x[0], x[1]))

        # Update persistent trace text
        trace_text.set_text(f"trace(P): {np.trace(P):.1f}")

        return drone_line, meas_scatter, est_point, cov_ellipse, trace_text

    ani = animation.FuncAnimation(
        fig, update, frames=num_steps, init_func=init, blit=False
    )

    # Try to save as mp4 using ffmpeg, fallback to gif via PillowWriter if not available
    try:
        writer = animation.FFMpegWriter(fps=15)
        ani.save(output_path, writer=writer)
        print(f"Saved animation to {output_path}")
    except Exception as e:
        print("FFmpeg writer failed:", e)
        gif_path = output_path.rsplit(".", 1)[0] + ".gif"
        try:
            writer = animation.PillowWriter(fps=15)
            ani.save(gif_path, writer=writer)
            print(f"Saved animation to {gif_path}")
        except Exception as e2:
            print("Failed to save animation with fallback writers:", e2)


if __name__ == "__main__":
    run_kf_video()
