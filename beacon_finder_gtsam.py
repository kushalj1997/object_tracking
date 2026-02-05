import math
import numpy as np
import gtsam
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse

"""Project: visualize covariance reduction for beacon localization using GTSAM.

This file runs a 2D beacon localization using GTSAM's NonlinearFactorGraph,
creating an MP4 animation showing the drone approaching the beacon with the
state covariance ellipse shrinking over time as measurements are added.
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


def cov_ellipse_params(P, nsig=2):
    """Return width, height, angle of covariance ellipse for 2x2 P (nsig sigma)."""
    vals, vecs = np.linalg.eigh(P)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    width, height = 2 * nsig * np.sqrt(vals)
    angle = math.degrees(math.atan2(vecs[1, 0], vecs[0, 0]))
    return width, height, angle


def run_gtsam_kf_video(output_path="beacon_covariance_gtsam.mp4", num_steps=200):
    """Run factor graph optimization at each step and visualize."""
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

    # GTSAM setup
    # Weak prior so measurements can drive the estimate
    noise_model_prior = gtsam.noiseModel.Isotropic.Sigma(2, 10000.0)
    # Tighter measurement noise to allow covariance to shrink visibly
    meas_sigma = 20.0
    noise_model_meas = gtsam.noiseModel.Isotropic.Sigma(2, meas_sigma)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlabel("ECEF X (m)")
    ax.set_ylabel("ECEF Y (m)")
    ax.set_title(
        "Beacon Localization (GTSAM) — Drone Approach with Covariance Shrinkage"
    )
    ax.axis("equal")

    # Set axis limits based on drone path and beacon
    all_points = np.vstack([drone_path, beacon_xy])
    margin = 2000
    ax.set_xlim(all_points[:, 0].min() - margin, all_points[:, 0].max() + margin)
    ax.set_ylim(all_points[:, 1].min() - margin, all_points[:, 1].max() + margin)

    (drone_line,) = ax.plot(
        [], [], "-", color="tab:blue", alpha=0.6, linewidth=1.5, label="Drone path"
    )
    (drone_pos,) = ax.plot(
        [], [], "o", color="tab:blue", markersize=8, label="Drone position"
    )
    meas_scatter = ax.scatter([], [], c="orange", s=20, alpha=0.6, label="Measurement")
    (est_point,) = ax.plot(
        [],
        [],
        "x",
        color="green",
        markersize=12,
        markeredgewidth=2,
        label="GTSAM estimate",
    )
    cov_ellipse = Ellipse(
        (0, 0),
        1,
        1,
        edgecolor="green",
        facecolor="green",
        alpha=0.15,
        lw=2,
        label="Covariance (2σ)",
    )
    ax.add_patch(cov_ellipse)
    ax.scatter(
        [beacon_xy[0]],
        [beacon_xy[1]],
        c="red",
        s=100,
        marker="*",
        label="True beacon",
        zorder=5,
    )
    ax.legend(loc="upper left")

    trace_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    def init():
        drone_line.set_data([], [])
        drone_pos.set_data([], [])
        meas_scatter.set_offsets(np.empty((0, 2)))
        est_point.set_data([], [])
        cov_ellipse.set_width(0)
        cov_ellipse.set_height(0)
        trace_text.set_text("")
        return drone_line, drone_pos, meas_scatter, est_point, cov_ellipse, trace_text

    def update(i):
        drone_so_far = drone_path[: i + 1]
        current_drone_pos = drone_path[i]

        # Build factor graph for this iteration
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate = gtsam.Values()

        # Beacon is at key 0
        beacon_key = gtsam.symbol("b", 0)

        # Prior on beacon (weak initially, gets stronger with measurements)
        beacon_prior = beacon_xy + np.array([10000.0, 10000.0])
        graph.add(
            gtsam.PriorFactorPoint2(
                beacon_key,
                gtsam.Point2(beacon_prior[0], beacon_prior[1]),
                noise_model_prior,
            )
        )
        initial_estimate.insert(
            beacon_key, gtsam.Point2(beacon_prior[0], beacon_prior[1])
        )

        # Add measurement factors for each drone position seen so far
        for j in range(i + 1):
            # Simulated measurement: noisy observation of beacon from drone j
            meas_noise = np.random.multivariate_normal(
                [0, 0], np.diag([meas_sigma**2, meas_sigma**2])
            )
            z = beacon_xy + meas_noise

            graph.add(
                gtsam.PriorFactorPoint2(
                    beacon_key,
                    gtsam.Point2(z[0], z[1]),
                    noise_model_meas,
                )
            )

        # Optimize
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
        try:
            result = optimizer.optimize()
        except Exception as e:
            print(f"Optimization failed at step {i}: {e}")
            result = initial_estimate

        # Extract beacon estimate and covariance
        try:
            beacon_est = result.atPoint2(beacon_key)
            x_est = np.array(beacon_est)  # atPoint2 returns a numpy array
        except Exception as e:
            print(f"Beacon est extraction failed at step {i}: {e}")
            x_est = beacon_xy

        # Extract covariance (2x2 submatrix for beacon position)
        try:
            marginals = gtsam.Marginals(graph, result)
            P = marginals.marginalCovariance(beacon_key)
        except Exception as e:
            print(f"Marginals failed at step {i}: {e}")
            P = np.diag([1e10, 1e10])  # fallback

        # Simulated measurement for display
        meas_noise = np.random.multivariate_normal(
            [0, 0], np.diag([meas_sigma**2, meas_sigma**2])
        )
        z = beacon_xy + meas_noise

        # Update plot elements
        drone_line.set_data(drone_so_far[:, 0], drone_so_far[:, 1])
        drone_pos.set_data([current_drone_pos[0]], [current_drone_pos[1]])
        meas_scatter.set_offsets(z.reshape(1, 2))
        est_point.set_data([x_est[0]], [x_est[1]])
        w, h, angle = cov_ellipse_params(P, nsig=2)
        cov_ellipse.set_width(w)
        cov_ellipse.set_height(h)
        cov_ellipse.set_angle(angle)
        cov_ellipse.set_center((x_est[0], x_est[1]))

        # Update trace text
        trace_val = np.trace(P)
        dist_to_beacon = np.linalg.norm(current_drone_pos - beacon_xy)
        trace_text.set_text(
            f"trace(P): {trace_val:.0f} | Distance to beacon: {dist_to_beacon:.0f} m"
        )

        return drone_line, drone_pos, meas_scatter, est_point, cov_ellipse, trace_text

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
    run_gtsam_kf_video()
