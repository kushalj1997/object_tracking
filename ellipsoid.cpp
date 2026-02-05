#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense> // for matrix manipulation

// Define a structure for 3D points
struct Point {
    double x, y, z;
};

// Function to generate points on an ellipsoid in spherical coordinates
std::vector<Point> generateEllipsoidPoints(double lambda1, double lambda2, double lambda3, int u_res, int v_res) {
    std::vector<Point> points;
    
    for (int i = 0; i < u_res; ++i) {
        for (int j = 0; j < v_res; ++j) {
            double u = 2 * M_PI * i / u_res;  // azimuthal angle (0 to 2*pi)
            double v = M_PI * j / (v_res - 1); // polar angle (0 to pi)

            // Parametric equations for ellipsoid
            double x = lambda1 * cos(u) * sin(v);
            double y = lambda2 * sin(u) * sin(v);
            double z = lambda3 * cos(v);

            points.push_back({x, y, z});
        }
    }
    
    return points;
}

// Function to project points onto a plane using a normal vector
std::vector<Point> projectOntoPlane(const std::vector<Point>& points, const Eigen::Vector3d& normal) {
    std::vector<Point> projectedPoints;
    
    Eigen::Matrix3d projectionMatrix = Eigen::Matrix3d::Identity() - normal * normal.transpose(); // Projection matrix

    for (const auto& point : points) {
        Eigen::Vector3d pt(point.x, point.y, point.z);
        Eigen::Vector3d projectedPt = projectionMatrix * pt; // Projecting point onto the plane
        projectedPoints.push_back({projectedPt(0), projectedPt(1), projectedPt(2)});
    }

    return projectedPoints;
}

int main() {
    double lambda1 = 2.0;  // semi-axis in x-direction
    double lambda2 = 1.0;  // semi-axis in y-direction
    double lambda3 = 1.5;  // semi-axis in z-direction
    int u_res = 100;       // resolution of u (0 to 2*pi)
    int v_res = 50;        // resolution of v (0 to pi)

    // Generate points on the ellipsoid
    std::vector<Point> ellipsoidPoints = generateEllipsoidPoints(lambda1, lambda2, lambda3, u_res, v_res);
    
    // Define the normal vector of the plane (e.g., z = 0 plane, normal is [0, 0, 1])
    Eigen::Vector3d normal(0, 0, 1); 
    
    // Project the ellipsoid points onto the plane
    std::vector<Point> projectedPoints = projectOntoPlane(ellipsoidPoints, normal);

    // Output some projected points (first 10 for brevity)
    for (int i = 0; i < 10; ++i) {
        std::cout << "Projected Point " << i << ": (" 
                  << projectedPoints[i].x << ", " 
                  << projectedPoints[i].y << ", " 
                  << projectedPoints[i].z << ")\n";
    }

    return 0;
}

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

// Function to generate points on the unit sphere
Eigen::Vector3d sampleUnitSphere()
{
    double theta = 2.0 * M_PI * (rand() / double(RAND_MAX)); // Azimuthal angle
    double phi = acos(2.0 * (rand() / double(RAND_MAX)) - 1.0); // Polar angle

    // Convert spherical coordinates to Cartesian coordinates (unit sphere)
    double x = sin(phi) * cos(theta);
    double y = sin(phi) * sin(theta);
    double z = cos(phi);

    return Eigen::Vector3d(x, y, z);
}

// Function to project 3D ellipsoid points to 2D
std::vector<Eigen::Vector2d> generateProjectedEllipsoidPoints(const Eigen::Matrix3d& covariance_matrix, int num_points)
{
    // Perform eigendecomposition of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance_matrix);
    Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors(); // Rotation matrix (eigenvectors)
    Eigen::Vector3d eigenvalues = eigensolver.eigenvalues().real(); // Scale factors (eigenvalues)

    std::vector<Eigen::Vector2d> ellipsoid_points_2d;
    
    // Generate points on the ellipsoid in 3D, then project them to 2D
    for (int i = 0; i < num_points; ++i)
    {
        // Sample a random point on the unit sphere
        Eigen::Vector3d unit_sphere_point = sampleUnitSphere();

        // Scale the point according to the eigenvalues (spread of the ellipsoid along the axes)
        Eigen::Vector3d scaled_point = eigenvectors * (unit_sphere_point.array() * eigenvalues.array().sqrt()).matrix();

        // Project the point to 2D by discarding the z-axis (third dimension)
        Eigen::Vector2d projected_point(scaled_point[0], scaled_point[1]);

        // Store the 2D projected point
        ellipsoid_points_2d.push_back(projected_point);
    }

    return ellipsoid_points_2d;
}

int main()
{
    // Define a sample 3D covariance matrix (3x3)
    Eigen::Matrix3d covariance_matrix;
    covariance_matrix << 1.0, 0.5, 0.2,
                         0.5, 1.0, 0.3,
                         0.2, 0.3, 1.0;

    // Number of points to generate on the 2D projected ellipsoid
    int num_points = 1000;

    // Generate ellipsoid points projected onto the 2D plane (visibility graph frame)
    std::vector<Eigen::Vector2d> ellipsoid_points_2d = generateProjectedEllipsoidPoints(covariance_matrix, num_points);

    // Print the first few points of the ellipsoid projection
    for (int i = 0; i < 10; ++i)
    {
        std::cout << "Projected Point " << i + 1 << ": " << ellipsoid_points_2d[i].transpose() << std::endl;
    }

    return 0;
}
