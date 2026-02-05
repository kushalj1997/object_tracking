import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse

"""Azimuth/Elevation/Range Radar Beacon Localization using Triangulation + Kalman Filter.

This file simulates a more realistic radar scenario:
- A radar on the drone measures azimuth, elevation, and range to the beacon.
- Triangulation computes the 3D beacon position from these spherical measurements.
- A Kalman filter fuses measurements and tracks the beacon with shrinking covariance.
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


def cartesian_to_az_el_range(target_xyz, radar_xyz):
    """
    Convert target position (3D) relative to radar position to azimuth, elevation, and range.

    Azimuth: angle in XY plane from North (Y), measured clockwise to East (X).
    Elevation: angle above the horizontal plane.
    Range: distance to target.

    Returns: (azimuth_rad, elevation_rad, range_m)
    """
    relative = target_xyz - radar_xyz
    dx, dy, dz = relative[0], relative[1], relative[2]

    # Azimuth: atan2(X, Y) gives angle from Y-axis, positive toward X
    az = math.atan2(dx, dy)

    # Elevation: angle above horizontal
    horizontal_dist = math.sqrt(dx**2 + dy**2)
    el = math.atan2(dz, horizontal_dist)

    # Range: Euclidean distance
    rng = np.linalg.norm(relative)

    return az, el, rng


def triangulate_from_az_el_range(
    radar_positions, az_measurements, el_measurements, range_measurements
):
    """
    Triangulate beacon position from multiple az/el/range measurements using Gauss-Newton.

    Args:
        radar_positions: list of [x, y, z] positions
        az_measurements: list of azimuth angles (radians)
        el_measurements: list of elevation angles (radians)
        range_measurements: list of range distances (meters)

    Returns:
        3D position [x, y, z] of beacon and estimated covariance scale
    """
    radar_positions = np.array(radar_positions)
    az_measurements = np.array(az_measurements)
    el_measurements = np.array(el_measurements)
    range_measurements = np.array(range_measurements)

    n_radars = len(radar_positions)
    if (
        n_radars != len(az_measurements)
        or n_radars != len(el_measurements)
        or n_radars != len(range_measurements)
    ):
        raise ValueError("Mismatched number of measurements")

    # Initial estimate: average of radar positions + range-weighted position
    beacon_est = np.mean(radar_positions, axis=0)
    for i in range(n_radars):
        direction = np.array(
            [
                math.sin(az_measurements[i]) * math.cos(el_measurements[i]),
                math.cos(az_measurements[i]) * math.cos(el_measurements[i]),
                math.sin(el_measurements[i]),
            ]
        )
        beacon_est += direction * (range_measurements[i] / n_radars)

    # Refine using Gauss-Newton iterations
    for iteration in range(20):
        residuals = []
        jacobians = []

        for i in range(n_radars):
            radar_pos = radar_positions[i]

            # Compute predicted measurements from current estimate
            az_pred, el_pred, rng_pred = cartesian_to_az_el_range(beacon_est, radar_pos)

            # Measurement residuals
            az_residual = az_measurements[i] - az_pred
            el_residual = el_measurements[i] - el_pred
            range_residual = range_measurements[i] - rng_pred

            residuals.extend([az_residual, el_residual, range_residual])

            # Jacobian: partial derivatives w.r.t. beacon_est
            relative = beacon_est - radar_pos
            dx, dy, dz = relative[0], relative[1], relative[2]
            horizontal_dist = math.sqrt(dx**2 + dy**2)
            rng = np.linalg.norm(relative)

            # daz/d(beacon) = [-dy/(dx^2+dy^2), dx/(dx^2+dy^2), 0]
            if horizontal_dist > 1e-6:
                daz_dx = -dy / (horizontal_dist**2)
                daz_dy = dx / (horizontal_dist**2)
                daz_dz = 0.0
            else:
                daz_dx = daz_dy = daz_dz = 0.0

            # del/d(beacon) = [-z*dx/(r^2*h), -z*dy/(r^2*h), h/r^2]
            if rng > 1e-6 and horizontal_dist > 1e-6:
                del_dx = -dz * dx / (rng**2 * horizontal_dist)
                del_dy = -dz * dy / (rng**2 * horizontal_dist)
                del_dz = horizontal_dist / (rng**2)
            else:
                del_dx = del_dy = del_dz = 0.0

            # drng/d(beacon) = relative / ||relative||
            if rng > 1e-6:
                drng_dx = dx / rng
                drng_dy = dy / rng
                drng_dz = dz / rng
            else:
                drng_dx = drng_dy = drng_dz = 0.0

            jacobians.append([daz_dx, daz_dy, daz_dz])
            jacobians.append([del_dx, del_dy, del_dz])
            jacobians.append([drng_dx, drng_dy, drng_dz])

        if not jacobians:
            break

        J = np.array(jacobians)
        r = np.array(residuals)

        # Gauss-Newton step: delta = inv(J^T J) * J^T * r
        try:
            JtJ = J.T @ J
            Jtr = J.T @ r
            delta = np.linalg.solve(JtJ, Jtr)
            beacon_est = beacon_est + delta

            if np.linalg.norm(delta) < 1e-4:
                break
        except:
            break

    # Estimate covariance scale from residuals
    residuals = []
    for i in range(n_radars):
        radar_pos = radar_positions[i]
        az_pred, el_pred, rng_pred = cartesian_to_az_el_range(beacon_est, radar_pos)
        residuals.extend(
            [
                az_measurements[i] - az_pred,
                el_measurements[i] - el_pred,
                range_measurements[i] - rng_pred,
            ]
        )

    residual_norm = np.linalg.norm(residuals)
    cov_scale = max(1.0, residual_norm / 10.0)

    return beacon_est, cov_scale


def cov_ellipse_params(P, nsig=2):
    """Return width, height, angle of covariance ellipse for 2x2 P (nsig sigma)."""
    vals, vecs = np.linalg.eigh(P)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    width, height = 2 * nsig * np.sqrt(np.maximum(vals, 0))
    angle = math.degrees(math.atan2(vecs[1, 0], vecs[0, 0]))
    return width, height, angle


def run_az_el_kf_video(output_path="beacon_covariance_az_el.mp4", num_steps=250):
    """Run azimuth/elevation radar beacon localization with Kalman filtering."""

    # Ground-truth beacon (3D)
    beacon_lla = LLA_Point(42.628356390253714, -83.10454313558542, 500)
    drone_start_lla = LLA_Point(42.63562993734775, -83.12015983135913, 400)
    beacon_ecef = np.array(lla_to_ecef(beacon_lla))
    drone_start_ecef = np.array(lla_to_ecef(drone_start_lla))

    # Drone flies in 3D: moves toward beacon in XY and also changes altitude
    drone_end_ecef = beacon_ecef.copy()
    drone_end_ecef[2] -= 50  # End 50m below beacon

    drone_path = np.array(
        [
            drone_start_ecef
            + (drone_end_ecef - drone_start_ecef) * (i / (num_steps - 1))
            for i in range(num_steps)
        ]
    )

    # Kalman Filter Setup
    # State: [x, y, z] position
    A = np.eye(3)
    C = np.eye(3)

    # Initial state estimate will be set from first triangulation
    x_est = None
    P = None
    Q = np.diag([1.0, 1.0, 1.0])

    # Measurement noise settings
    az_sigma_high = math.radians(10.0)
    el_sigma_high = math.radians(10.0)
    range_sigma_high = 500.0  # meters
    az_sigma_low = math.radians(0.5)
    el_sigma_low = math.radians(0.5)
    range_sigma_low = 5.0  # meters
    initial_high_frames = int(num_steps * 0.5)

    # Visualization setup
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111)
    ax.set_xlabel("ECEF X (m)")
    ax.set_ylabel("ECEF Y (m)")
    ax.set_title("Azimuth/Elevation Radar Beacon (Triangulation + KF)")
    ax.axis("equal")

    all_points = np.vstack([drone_path, beacon_ecef])
    margin = 1000
    ax.set_xlim(all_points[:, 0].min() - margin, all_points[:, 0].max() + margin)
    ax.set_ylim(all_points[:, 1].min() - margin, all_points[:, 1].max() + margin)

    (drone_line,) = ax.plot(
        [], [], "-", color="tab:blue", alpha=0.6, linewidth=1.5, label="Drone path"
    )
    (drone_pos,) = ax.plot(
        [], [], "o", color="tab:blue", markersize=8, label="Drone position"
    )
    (est_point,) = ax.plot(
        [],
        [],
        "x",
        color="green",
        markersize=12,
        markeredgewidth=2,
        label="KF estimate (xy)",
    )

    cov_ellipse = Ellipse(
        (0, 0),
        1,
        1,
        edgecolor="green",
        facecolor="none",
        alpha=0.9,
        lw=3,
        label="Covariance (2σ)",
    )
    ax.add_patch(cov_ellipse)

    ax.scatter(
        [beacon_ecef[0]],
        [beacon_ecef[1]],
        c="red",
        s=100,
        marker="*",
        label="True beacon",
        zorder=5,
    )
    ax.legend(loc="upper left")

    trace_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)
    trace_history = []
    error_history = []

    def init():
        drone_line.set_data([], [])
        drone_pos.set_data([], [])
        est_point.set_data([], [])
        cov_ellipse.set_width(0)
        cov_ellipse.set_height(0)
        trace_text.set_text("")
        return drone_line, drone_pos, est_point, cov_ellipse, trace_text

    def update(i):
        nonlocal x_est, P

        drone_so_far = drone_path[: i + 1]
        current_drone_pos = drone_path[i]

        # Collect measurements from all drone positions
        radar_positions = drone_so_far.copy()
        az_measurements = []
        el_measurements = []
        range_measurements = []

        for radar_pos in radar_positions:
            if len(radar_positions) <= initial_high_frames:
                az_sigma = az_sigma_high
                el_sigma = el_sigma_high
                range_sigma = range_sigma_high
            else:
                az_sigma = az_sigma_low
                el_sigma = el_sigma_low
                range_sigma = range_sigma_low

            az_true, el_true, range_true = cartesian_to_az_el_range(
                beacon_ecef, radar_pos
            )
            az_meas = az_true + np.random.normal(0, az_sigma)
            el_meas = el_true + np.random.normal(0, el_sigma)
            range_meas = range_true + np.random.normal(0, range_sigma)

            az_measurements.append(az_meas)
            el_measurements.append(el_meas)
            range_measurements.append(range_meas)

        # Triangulate
        try:
            z_tri, cov_scale = triangulate_from_az_el_range(
                radar_positions, az_measurements, el_measurements, range_measurements
            )
        except Exception as e:
            z_tri = x_est.copy() if x_est is not None else np.zeros(3)
            cov_scale = 1.0

        # Initialize Kalman filter on first frame with triangulation result
        if x_est is None:
            x_est = z_tri.copy()
            # Start with covariance scaled to triangulation uncertainty
            P = np.eye(3) * (cov_scale**2 * 10000.0)

        # Kalman update
        x_pred = A @ x_est
        P_pred = A @ P @ A.T + Q

        y = z_tri - C @ x_pred
        S = C @ P_pred @ C.T + np.eye(3) * (cov_scale**2 * 1000.0)
        K = P_pred @ C.T @ np.linalg.inv(S)

        x_est = x_pred + K @ y
        P = (np.eye(3) - K @ C) @ P_pred

        # Compute error and store history
        est_error = np.linalg.norm(x_est - beacon_ecef)
        error_history.append(float(est_error))

        # Store trace
        trace_val = np.trace(P[:2, :2])
        trace_history.append(float(trace_val))

        # Update plot
        drone_line.set_data(drone_so_far[:, 0], drone_so_far[:, 1])
        drone_pos.set_data([current_drone_pos[0]], [current_drone_pos[1]])
        est_point.set_data([x_est[0]], [x_est[1]])

        P_xy = P[:2, :2]
        w, h, angle = cov_ellipse_params(P_xy, nsig=2)
        cov_ellipse.set_width(w)
        cov_ellipse.set_height(h)
        cov_ellipse.set_angle(angle)
        cov_ellipse.set_center((x_est[0], x_est[1]))

        if i < initial_high_frames:
            az_sig_display = math.degrees(az_sigma_high)
            el_sig_display = math.degrees(el_sigma_high)
            range_sig_display = range_sigma_high
        else:
            az_sig_display = math.degrees(az_sigma_low)
            el_sig_display = math.degrees(el_sigma_low)
            range_sig_display = range_sigma_low

        # Calculate error improvement from first estimate
        error_improvement = ""
        if len(error_history) > 1:
            error_change = error_history[0] - est_error
            error_improvement = f" | Improved: {error_change:.1f}m"

        trace_text.set_text(
            f"trace(P_xy): {trace_val:.0f} | Est.error: {est_error:.1f}m{error_improvement} | "
            f"Az/El/Rng noise: {az_sig_display:.1f}°/{el_sig_display:.1f}°/{range_sig_display:.1f}m"
        )

        return drone_line, drone_pos, est_point, cov_ellipse, trace_text

    ani = animation.FuncAnimation(
        fig, update, frames=num_steps, init_func=init, blit=False
    )

    # Save
    try:
        writer = animation.FFMpegWriter(fps=15)
        ani.save(output_path, writer=writer)
        print(f"Saved animation to {output_path}")
    except Exception as e:
        print("FFmpeg failed:", e)
        gif_path = output_path.rsplit(".", 1)[0] + ".gif"
        try:
            writer = animation.PillowWriter(fps=15)
            ani.save(gif_path, writer=writer)
            print(f"Saved animation to {gif_path}")
        except Exception as e2:
            print("Failed to save:", e2)

    # Trace
    try:
        with open(output_path.rsplit(".", 1)[0] + "_trace.txt", "w") as fh:
            for v in trace_history:
                fh.write(f"{v}\n")
        print("Wrote trace history to", output_path.rsplit(".", 1)[0] + "_trace.txt")
    except Exception as e:
        print("Failed to write trace:", e)


if __name__ == "__main__":
    run_az_el_kf_video()
