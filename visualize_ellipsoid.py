import numpy as np
import matplotlib.pyplot as plt


# Function to generate random points on the unit sphere
def sample_unit_sphere(num_points):
    points = []
    for _ in range(num_points):
        theta = 2.0 * np.pi * np.random.rand()  # Azimuthal angle
        phi = np.arccos(2.0 * np.random.rand() - 1.0)  # Polar angle

        # Convert spherical coordinates to Cartesian coordinates
        x = np.sin(phi) * np.cos(theta)
        y = np.sin(phi) * np.sin(theta)
        z = np.cos(phi)
        points.append(np.array([x, y, z]))
    return np.array(points)


# Function to generate the ellipsoid points from the covariance matrix
def generate_ellipsoid_points(cov_matrix, num_points):
    # Eigenvalue decomposition of the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

    # Sample points on the unit sphere
    unit_sphere_points = sample_unit_sphere(num_points)

    # Transform points from unit sphere to the ellipsoid
    ellipsoid_points = np.dot(unit_sphere_points, eigenvectors) * np.sqrt(eigenvalues)
    return ellipsoid_points


# Project the 3D ellipsoid points onto the 2D plane (XY plane in this case)
def project_to_2d(ellipsoid_points):
    return ellipsoid_points[:, :2]


# Generate boundary points around the ellipsoid's projection (assumed circular for simplicity)
def generate_boundary_points(cov_matrix, num_points):
    # Eigenvalue decomposition of the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

    # Sample evenly spaced points on the boundary (a circle for simplicity)
    theta = np.linspace(0, 2 * np.pi, num_points)

    # Generate points along the boundary of the ellipsoid in 2D
    boundary_points = np.zeros((num_points, 2))  # Initialize as 2D points (XY)
    for i, angle in enumerate(theta):
        # Create the point in the eigenvector space
        point = np.array(
            [
                np.cos(angle) * np.sqrt(eigenvalues[0]),  # X component
                np.sin(angle) * np.sqrt(eigenvalues[1]),  # Y component
                0.0,  # Z component (we're working in 2D)
            ]
        )
        # Transform back to the original space using the eigenvectors (account for rotation)
        transformed_point = np.dot(eigenvectors, point)
        # Store only the XY components
        boundary_points[i] = transformed_point[:2]
    return boundary_points


# Function to create a simple visibility graph
def create_visibility_graph(
    points, threshold=1.0, start_point=None, end_point=None, boundary_points=None
):
    edges = []

    # Add edges from start point to the boundary points
    if start_point is not None:
        for i, point in enumerate(boundary_points):
            dist = np.linalg.norm(start_point - point)
            if dist < threshold:
                edges.append(("start", i))  # Connect start point to the boundary points

    # Add edges from end point to the boundary points
    if end_point is not None:
        for i, point in enumerate(boundary_points):
            dist = np.linalg.norm(end_point - point)
            if dist < threshold:
                edges.append(("end", i))  # Connect end point to the boundary points

    # Add edges between points in the boundary based on proximity
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            dist = np.linalg.norm(points[i] - points[j])
            if dist < threshold:
                edges.append((i, j))

    return edges


# Function to generate start and end points such that their direct path intersects the ellipsoid
def generate_outside_points(
    ellipsoid_points, boundary_points, region_size, num_points=2
):
    # First, find the center of the ellipsoid (mean of boundary points)
    center = np.mean(boundary_points, axis=0)

    # Generate random angle for the line that will pass through the ellipsoid
    angle = np.random.uniform(0, 2 * np.pi)
    direction = np.array([np.cos(angle), np.sin(angle)])

    # Find the maximum distance from center to boundary points
    max_dist = np.max(np.linalg.norm(boundary_points - center, axis=1))

    # Generate points far enough from the center but not too far
    # We want them to be outside the ellipsoid but within the region
    min_dist = max_dist * 1.5  # At least 1.5 times the max radius
    max_dist = min(region_size / 2, max_dist * 3)  # Not too far, but within region

    # Generate start point
    start_dist = np.random.uniform(min_dist, max_dist)
    start_point = center + direction * start_dist

    # Generate end point in the opposite direction
    end_dist = np.random.uniform(min_dist, max_dist)
    end_point = center - direction * end_dist

    # Verify that the points are within the region
    if np.any(np.abs(start_point) > region_size / 2) or np.any(
        np.abs(end_point) > region_size / 2
    ):
        # If points are outside region, try again
        return generate_outside_points(
            ellipsoid_points, boundary_points, region_size, num_points
        )

    return start_point, end_point


# Function to find the shortest path through the visibility graph
def find_shortest_path(edges, boundary_points, start_point, end_point):
    # Create a graph representation
    graph = {}
    # Add start and end points to the graph
    graph["start"] = []
    graph["end"] = []

    # Calculate the center of the ellipsoid
    center = np.mean(boundary_points, axis=0)

    # Calculate the maximum distance from center to boundary points
    max_dist = np.max(np.linalg.norm(boundary_points - center, axis=1))
    min_safe_dist = max_dist * 0.8  # Minimum safe distance from center

    # Add all boundary points to the graph
    for i in range(len(boundary_points)):
        graph[i] = []

    # Build the graph from edges with center-penalized weights
    start_connections = 0
    end_connections = 0
    for edge in edges:
        if edge[0] == "start":
            # Calculate distance from center to the midpoint of start-boundary connection
            mid_point = (start_point + boundary_points[edge[1]]) / 2
            dist_to_center = np.linalg.norm(mid_point - center)
            # Add penalty if the path gets too close to center
            penalty = 1.0
            if dist_to_center < min_safe_dist:
                penalty = 1.0 + (min_safe_dist - dist_to_center) / min_safe_dist * 10.0
            weight = np.linalg.norm(start_point - boundary_points[edge[1]]) * penalty
            graph["start"].append((edge[1], weight))
            start_connections += 1
        elif edge[0] == "end":
            # Calculate distance from center to the midpoint of end-boundary connection
            mid_point = (end_point + boundary_points[edge[1]]) / 2
            dist_to_center = np.linalg.norm(mid_point - center)
            # Add penalty if the path gets too close to center
            penalty = 1.0
            if dist_to_center < min_safe_dist:
                penalty = 1.0 + (min_safe_dist - dist_to_center) / min_safe_dist * 10.0
            weight = np.linalg.norm(end_point - boundary_points[edge[1]]) * penalty
            graph[edge[1]].append(("end", weight))
            end_connections += 1
        else:
            i, j = edge
            # Calculate distance from center to the midpoint of the edge
            mid_point = (boundary_points[i] + boundary_points[j]) / 2
            dist_to_center = np.linalg.norm(mid_point - center)
            # Add penalty if the path gets too close to center
            penalty = 1.0
            if dist_to_center < min_safe_dist:
                penalty = 1.0 + (min_safe_dist - dist_to_center) / min_safe_dist * 10.0
            weight = np.linalg.norm(boundary_points[i] - boundary_points[j]) * penalty
            graph[i].append((j, weight))
            graph[j].append((i, weight))

    print(f"Start point connections: {start_connections}")
    print(f"End point connections: {end_connections}")
    print(f"Minimum safe distance from center: {min_safe_dist:.2f}")

    if start_connections == 0 or end_connections == 0:
        print("Warning: Start or end point has no connections!")
        return []

    # Dijkstra's algorithm
    distances = {node: float("infinity") for node in graph}
    distances["start"] = 0
    previous = {node: None for node in graph}
    unvisited = set(graph.keys())

    while unvisited:
        # Find the unvisited node with the smallest distance
        current = min(unvisited, key=lambda x: distances[x])
        if current == "end":
            break

        if distances[current] == float("infinity"):
            print("Warning: No path to end point found!")
            return []

        unvisited.remove(current)

        # Update distances to neighbors
        for neighbor, weight in graph[current]:
            if neighbor in unvisited:
                new_distance = distances[current] + weight
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = current

    if distances["end"] == float("infinity"):
        print("Warning: No path to end point found!")
        return []

    # Reconstruct the path
    path = []
    current = "end"
    while current is not None:
        if current != "start" and current != "end":
            path.append(boundary_points[current])
        current = previous[current]

    if not path:
        print("Warning: Empty path after reconstruction!")
    else:
        print(f"Path found with {len(path)} points")
        # Calculate minimum distance from center to any point in the path
        path_points = np.array(path)
        min_dist_to_center = np.min(np.linalg.norm(path_points - center, axis=1))
        print(f"Minimum distance from center to path: {min_dist_to_center:.2f}")

    return path[::-1]  # Reverse the path to go from start to end


# Function to plot the results in 3D and 2D
def plot_graph(
    ellipsoid_points_2d,
    ellipsoid_points_3d,
    edges,
    start_point,
    end_point,
    boundary_points_2d,
):
    # Find the shortest path through the visibility graph
    path = find_shortest_path(edges, boundary_points_2d, start_point, end_point)

    # Create figure with two subplots
    fig = plt.figure(figsize=(14, 7))

    # 3D plot (left subplot)
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.scatter(
        ellipsoid_points_3d[:, 0],
        ellipsoid_points_3d[:, 1],
        ellipsoid_points_3d[:, 2],
        c="blue",
        alpha=0.3,  # Make 3D points more transparent
        label="3D Ellipsoid",
    )
    ax1.set_xlabel("X-axis")
    ax1.set_ylabel("Y-axis")
    ax1.set_zlabel("Z-axis")
    ax1.set_title("3D Ellipsoid in World Frame")

    # 2D plot (right subplot)
    ax2 = fig.add_subplot(122)

    # Plot the 2D projection of the ellipsoid
    ax2.scatter(
        ellipsoid_points_2d[:, 0],
        ellipsoid_points_2d[:, 1],
        c="blue",
        alpha=0.1,  # Make ellipsoid points very transparent
        label="2D Projection",
    )

    # Plot the boundary points (nodes of the visibility graph)
    ax2.scatter(
        boundary_points_2d[:, 0],
        boundary_points_2d[:, 1],
        c="orange",
        s=30,  # Smaller points for boundary nodes
        label="Boundary Nodes",
        zorder=5,
    )

    # Plot the visibility graph edges
    for edge in edges:
        if edge[0] == "start":
            ax2.plot(
                [start_point[0], boundary_points_2d[edge[1]][0]],
                [start_point[1], boundary_points_2d[edge[1]][1]],
                "g--",
                alpha=0.2,  # Very transparent for visibility graph
                label="Visibility Graph" if edge[1] == 0 else "",
            )
        elif edge[0] == "end":
            ax2.plot(
                [end_point[0], boundary_points_2d[edge[1]][0]],
                [end_point[1], boundary_points_2d[edge[1]][1]],
                "g--",
                alpha=0.2,
            )
        else:
            ax2.plot(
                [boundary_points_2d[edge[0]][0], boundary_points_2d[edge[1]][0]],
                [boundary_points_2d[edge[0]][1], boundary_points_2d[edge[1]][1]],
                "g--",
                alpha=0.2,
            )

    # Plot the start and end points
    ax2.scatter(
        start_point[0],
        start_point[1],
        c="green",
        s=100,  # Larger points for start/end
        label="Start Point",
        zorder=5,
    )
    ax2.scatter(
        end_point[0],
        end_point[1],
        c="red",
        s=100,  # Larger points for start/end
        label="End Point",
        zorder=5,
    )

    # Plot the actual path that goes around the ellipsoid
    if path:
        # Add start and end points to the path
        full_path = [start_point] + path + [end_point]
        path_x = [p[0] for p in full_path]
        path_y = [p[1] for p in full_path]
        ax2.plot(
            path_x,
            path_y,
            "r-",
            linewidth=2,
            label="Path Around Ellipsoid",
            zorder=4,  # Ensure path is above ellipsoid points but below nodes
        )

    # Set equal aspect ratio for 2D plot
    ax2.set_aspect("equal")
    ax2.set_xlabel("X-axis")
    ax2.set_ylabel("Y-axis")
    ax2.set_title("2D Visibility Graph and Path")

    # Add legend only to 2D plot
    ax2.legend(loc="upper right", bbox_to_anchor=(1.15, 1))

    # Adjust layout to prevent legend cutoff
    plt.tight_layout()
    plt.show()


# Main function
def main():
    # Define a 3x3 covariance matrix (1km x 1km x 1km covariance in ECEF coordinates)
    covariance_matrix = np.array(
        [[1000000.0, 0.5, 0.2], [0.5, 1000000.0, 0.3], [0.2, 0.3, 1000000.0]]
    )

    # Region of operation (10km x 10km x 10km)
    region_size = 10000  # in meters (10km)

    # Number of points on the ellipsoid
    num_points = 1000

    # Generate ellipsoid points (3D) and then project to 2D
    ellipsoid_points_3d = generate_ellipsoid_points(covariance_matrix, num_points)

    # Project the ellipsoid points onto the 2D plane (XY plane)
    ellipsoid_points_2d = project_to_2d(ellipsoid_points_3d)

    # Generate boundary points around the ellipsoid's projection
    boundary_points_2d = generate_boundary_points(covariance_matrix, num_points=100)

    # Generate start and end points such that their direct path intersects the ellipsoid
    start_point, end_point = generate_outside_points(
        ellipsoid_points_2d, boundary_points_2d, region_size
    )

    # Create the visibility graph with a larger threshold to ensure we have enough connections
    edges = create_visibility_graph(
        boundary_points_2d,
        threshold=1000,  # Increased threshold to ensure more connections
        start_point=start_point,
        end_point=end_point,
        boundary_points=boundary_points_2d,
    )

    # Debug prints
    print(f"Number of boundary points: {len(boundary_points_2d)}")
    print(f"Number of edges in visibility graph: {len(edges)}")

    # Find the shortest path
    path = find_shortest_path(edges, boundary_points_2d, start_point, end_point)
    print(f"Path length: {len(path)}")
    if len(path) == 0:
        print("Warning: No path found! Adjusting visibility graph...")
        # Try again with an even larger threshold
        edges = create_visibility_graph(
            boundary_points_2d,
            threshold=2000,  # Further increased threshold
            start_point=start_point,
            end_point=end_point,
            boundary_points=boundary_points_2d,
        )
        path = find_shortest_path(edges, boundary_points_2d, start_point, end_point)
        print(f"New number of edges: {len(edges)}")
        print(f"New path length: {len(path)}")

    # Plot the visibility graph and the projected ellipsoid
    plot_graph(
        ellipsoid_points_2d,
        ellipsoid_points_3d,
        edges,
        start_point,
        end_point,
        boundary_points_2d,
    )


if __name__ == "__main__":
    main()
