#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <climits>
#include <algorithm>

// Point structure to represent grid coordinates
struct Point {
    int x, y;

    Point(int x = 0, int y = 0) : x(x), y(y) {}

    // Manhattan distance (since we're using grid coordinates)
    int distance(const Point& other) const {
        return std::abs(x - other.x) + std::abs(y - other.y);
    }
};

// Edge structure to represent a connection between two points
struct Edge {
    int from, to;
    double weight;
    
    Edge(int f, int t, double w) : from(f), to(t), weight(w) {}
};

// Visibility Graph Class
class VisibilityGraph {
public:
    // List of nodes (points in the grid)
    std::vector<Point> nodes;
    // List of edges (connections between points)
    std::vector<Edge> edges;

    // 2D Grid representing the environment (0 for free, 1 for obstacle)
    std::vector<std::vector<int>> grid;
    int rows, cols;

    // Constructor to initialize grid dimensions
    VisibilityGraph(int rows, int cols) : rows(rows), cols(cols) {
        grid.resize(rows, std::vector<int>(cols, 0));  // Initialize grid to all free space (0)
    }

    // Add a node to the graph
    void addNode(int x, int y) {
        nodes.push_back(Point(x, y));
    }

    // Add an edge to the graph
    void addEdge(int from, int to, double weight) {
        edges.push_back(Edge(from, to, weight));
    }

    // Bresenham's Line Algorithm to check if there is a clear line of sight
    bool isVisible(int i, int j) {
        const Point& p1 = nodes[i];
        const Point& p2 = nodes[j];

        // Bresenham's line algorithm to check for obstacles
        int dx = std::abs(p2.x - p1.x);
        int dy = std::abs(p2.y - p1.y);
        int sx = (p1.x < p2.x) ? 1 : -1;
        int sy = (p1.y < p2.y) ? 1 : -1;
        int err = dx - dy;

        int x = p1.x, y = p1.y;
        while (true) {
            if (x == p2.x && y == p2.y)
                break;
            if (grid[y][x] == 1) // Check if there's an obstacle at (x, y)
                return false;

            int e2 = err * 2;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
        return true;
    }

    // Build the visibility graph by connecting all visible points
    void buildGraph() {
        for (int i = 0; i < nodes.size(); ++i) {
            for (int j = i + 1; j < nodes.size(); ++j) {
                if (isVisible(i, j)) {
                    double weight = nodes[i].distance(nodes[j]);
                    addEdge(i, j, weight);
                    addEdge(j, i, weight); // Since the graph is undirected
                }
            }
        }
    }
};

// Dijkstra's algorithm to find the shortest path
std::vector<int> dijkstra(const VisibilityGraph& graph, int start, int goal) {
    int n = graph.nodes.size();
    std::vector<double> dist(n, INT_MAX);
    std::vector<int> prev(n, -1);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) break;

        for (const Edge& edge : graph.edges) {
            if (edge.from == u || edge.to == u) {
                int v = (edge.from == u) ? edge.to : edge.from;
                double weight = edge.weight;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
    }

    // Reconstruct the path
    std::vector<int> path;
    for (int at = goal; at != -1; at = prev[at]) {
        path.push_back(at);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

// Main function to run the visibility graph and path planning algorithm
int main() {
    // Create a 10x10 grid (0 is free space, 1 is an obstacle)
    VisibilityGraph graph(10, 10);

    // Add obstacles (1 indicates an obstacle at the given coordinate)
    graph.grid[3][4] = 1;
    graph.grid[4][4] = 1;
    graph.grid[5][4] = 1;
    graph.grid[6][6] = 1;

    // Add nodes (points in the grid)
    graph.addNode(0, 0);  // Start
    graph.addNode(9, 9);  // Goal
    graph.addNode(2, 3);  // A point
    graph.addNode(7, 8);  // Another point

    // Build the visibility graph by connecting all visible points
    graph.buildGraph();

    // Find the shortest path using Dijkstra's algorithm
    std::vector<int> path = dijkstra(graph, 0, 1);

    // Print the path
    std::cout << "Shortest Path: ";
    for (int index : path) {
        std::cout << "(" << graph.nodes[index].x << ", " << graph.nodes[index].y << ") ";
    }
    std::cout << std::endl;

    return 0;
}
