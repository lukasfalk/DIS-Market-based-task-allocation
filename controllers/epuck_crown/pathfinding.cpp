#include "pathfinding.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

// --- Line Segment Intersection and Helper Functions ---

/// @brief Determines the orientation of an ordered triplet of points (p, q, r).
/// @return 0 if collinear, 1 if clockwise, 2 if counterclockwise
static int orientation(Point2d p, Point2d q, Point2d r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    // Use epsilon for floating-point comparison
    const double epsilon = 1e-9;
    if (std::abs(val) < epsilon) return 0;  // Collinear
    return (val > 0) ? 1 : 2;               // Clockwise or Counterclockwise
}

/// @brief Checks if point q lies on line segment pr (given that p, q, r are collinear).
/// @return true if q lies on segment pr, false otherwise
static bool onSegment(Point2d p, Point2d q, Point2d r) {
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) {
        return true;
    }
    return false;
}

/// @brief Checks if two line segments 'p1q1' and 'p2q2' intersect.
/// @details Uses strict interior intersection detection. Endpoints touching or being
///          collinear with walls do NOT count as intersections (allowing paths along walls).
/// @returns true only if segments cross in their interiors, false otherwise
static bool doSegmentsIntersect(Point2d p1, Point2d q1, Point2d p2, Point2d q2) {
    // Early exit: if any endpoint of the wall segment is an endpoint of the path, they just touch
    // This allows paths to slide along walls
    const double eps = 1e-9;
    auto pointsEqual = [eps](Point2d a, Point2d b) {
        return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps;
    };

    if (pointsEqual(p1, p2) || pointsEqual(p1, q2) || pointsEqual(q1, p2) || pointsEqual(q1, q2)) {
        return false;  // Endpoints touch - not a blocking intersection
    }

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case: segments intersect if endpoints are on strictly opposite sides
    if (o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0 && o1 != o2 && o3 != o4) {
        return true;
    }

    // Collinear cases: only count as intersection if a point is strictly INSIDE the other segment
    // (not at either endpoint) which would indicate the path passes through the wall interior
    if (o1 == 0 && onSegment(p1, p2, q1) && !pointsEqual(p2, p1) && !pointsEqual(p2, q1)) {
        return true;
    }

    if (o2 == 0 && onSegment(p1, q2, q1) && !pointsEqual(q2, p1) && !pointsEqual(q2, q1)) {
        return true;
    }

    if (o3 == 0 && onSegment(p2, p1, q2) && !pointsEqual(p1, p2) && !pointsEqual(p1, q2)) {
        return true;
    }

    if (o4 == 0 && onSegment(p2, q1, q2) && !pointsEqual(q1, p2) && !pointsEqual(q1, q2)) {
        return true;
    }

    return false;  // No blocking intersection found
}

PathPlanner::PathPlanner() {
    // --- 1. Define the Static Graph Nodes ---
    // Hard-code nodes using coordinates from the Webots scene.
    /* Spatially, the nodes are arranged like this:

        A ------------ B C ------ D  (top row)
        |              | |        |
        |              | |        |
        |              | |        |
        E ----- F      | |        |  (second row)
        G ----- H      | |        |  (third row)
        |              I J        |  (fourth row)
        |                         |
        |                         |
        K ----------------------- L  (bottom row)
    */
    nodes = {
        // Top row
        {'A', {-0.575, 0.575}},  // A = 0
        {'B', {0.07, 0.575}},    // B = 1
        {'C', {0.18, 0.575}},    // C = 2
        {'D', {0.575, 0.575}},   // D = 3
        // Second row
        {'E', {-0.575, 0.055}},  // E = 4
        {'F', {-0.215, 0.055}},  // F = 5
        // Third row
        {'G', {-0.575, -0.055}},  // G = 6
        {'H', {-0.215, -0.055}},  // H = 7
        // Fourth row
        {'I', {0.07, -0.25}},  // I = 8
        {'J', {0.18, -0.25}},  // J = 9
        // Bottom row
        {'K', {-0.575, -0.575}},  // K = 10
        {'L', {0.575, -0.575}},   // L = 11
    };

    // --- 2. Define the Wall Obstacles ---
    // Using the same coordinates as above for inner wall nodes.
    // NOTE: We define walls as the PERIMETER of rectangular obstacles.
    // This ensures paths cannot pass through obstacle interiors.
    // Additionally, we add INTERIOR CENTERLINE segments that pass through the middle
    // of each obstacle to prevent nodes on the same edge from seeing each other through
    // the obstacle interior (without needing wall thickness/normal information).
    walls = {
        // Outer boundary walls
        {nodes[0].pos, nodes[3].pos},    // Top wall (edge A-D)
        {nodes[0].pos, nodes[10].pos},   // Left wall (edge A-K)
        {nodes[10].pos, nodes[11].pos},  // Bottom wall (edge K-L)
        {nodes[3].pos, nodes[11].pos},   // Right wall (edge D-L)

        // Inner vertical wall BIJC
        {nodes[1].pos, nodes[8].pos},  // Left segment (edge B-I)
        {nodes[8].pos, nodes[9].pos},  // Bottom segment (edge I-J)
        {nodes[9].pos, nodes[2].pos},  // Right segment (edge J-C)
        // Interior centerline segment from midpoint(B-C) to midpoint(I-J), slightly inset to block same-wall nodes from seeing each other
        {Point2d((nodes[1].pos.x + nodes[2].pos.x) / 2.0, (nodes[1].pos.y + nodes[2].pos.y) / 2.0 + 0.01),
         Point2d((nodes[8].pos.x + nodes[9].pos.x) / 2.0, (nodes[8].pos.y + nodes[9].pos.y) / 2.0 + 0.01)},

        // Inner horizontal wall EFHG
        {nodes[4].pos, nodes[5].pos},  // Top segment (edge E-F)
        {nodes[5].pos, nodes[7].pos},  // Right segment (edge F-H)
        {nodes[7].pos, nodes[6].pos},  // Bottom segment (edge H-G)
        // Interior centerline segment from midpoint(E-G) to midpoint(F-H), slightly inset to block same-wall nodes from seeing each other
        {Point2d((nodes[4].pos.x + nodes[6].pos.x) / 2.0 - 0.01, (nodes[4].pos.y + nodes[6].pos.y) / 2.0),
         Point2d((nodes[5].pos.x + nodes[7].pos.x) / 2.0 - 0.01, (nodes[5].pos.y + nodes[7].pos.y) / 2.0)},
    };

    // --- 3. Pre-compute the Adjacency Matrix ---
    int num_nodes = nodes.size();
    adj_matrix.resize(num_nodes, std::vector<double>(num_nodes, -1.0));

    for (int i = 0; i < num_nodes; ++i) {
        for (int j = i; j < num_nodes; ++j) {
            if (i == j) {
                adj_matrix[i][j] = 0;
                continue;
            }
            // Check for line of sight between every pair of nodes
            bool has_los = hasLineOfSight(nodes[i].pos, nodes[j].pos);
            if (has_los) {
                double dist = nodes[i].pos.Distance(nodes[j].pos);
                adj_matrix[i][j] = dist;
                adj_matrix[j][i] = dist;  // The graph is undirected so ij = ji
            }
        }
    }
}

/// @brief Finds the shortest path from a start to a goal using A* search.
/// @details This method augments the visibility graph by temporarily adding the start and goal
///          points as nodes and connecting them to any existing nodes they have line-of-sight with.
///          Then it runs A* to find the shortest path. The temporary connections are not persisted.
/// @param start The starting point in the environment
/// @param goal The goal point in the environment
/// @return A vector of points representing the path from start to goal, or empty vector if no path exists
std::vector<Point2d> PathPlanner::findPath(Point2d start, Point2d goal) {
    int num_nodes = nodes.size();

    // Create a temporary extended adjacency matrix that includes start and goal as nodes
    // Indices: 0 to num_nodes-1 are original nodes, num_nodes is start, num_nodes+1 is goal
    std::vector<std::vector<double>> extended_adj_matrix(num_nodes + 2, std::vector<double>(num_nodes + 2, -1.0));

    // Copy the original adjacency matrix
    for (int i = 0; i < num_nodes; ++i) {
        for (int j = 0; j < num_nodes; ++j) {
            extended_adj_matrix[i][j] = adj_matrix[i][j];
        }
    }

    int start_idx = num_nodes;
    int goal_idx = num_nodes + 1;

    // Self-loops: distance from a node to itself is 0
    extended_adj_matrix[start_idx][start_idx] = 0;
    extended_adj_matrix[goal_idx][goal_idx] = 0;

    // Connect start node to all nodes with line-of-sight
    for (int i = 0; i < num_nodes; ++i) {
        if (hasLineOfSight(start, nodes[i].pos)) {
            double dist = start.Distance(nodes[i].pos);
            extended_adj_matrix[start_idx][i] = dist;
            extended_adj_matrix[i][start_idx] = dist;
        }
    }

    // Connect goal node to all nodes with line-of-sight
    for (int i = 0; i < num_nodes; ++i) {
        if (hasLineOfSight(goal, nodes[i].pos)) {
            double dist = goal.Distance(nodes[i].pos);
            extended_adj_matrix[goal_idx][i] = dist;
            extended_adj_matrix[i][goal_idx] = dist;
        }
    }

    // Also check for direct line-of-sight between start and goal
    if (hasLineOfSight(start, goal)) {
        double dist = start.Distance(goal);
        extended_adj_matrix[start_idx][goal_idx] = dist;
        extended_adj_matrix[goal_idx][start_idx] = dist;
    }

    // Temporarily extend the nodes vector to include start and goal
    GraphNode start_node = {'S', start};
    GraphNode goal_node = {'G', goal};
    nodes.push_back(start_node);
    nodes.push_back(goal_node);

    // Run A* from start to goal using the extended graph
    std::vector<Point2d> path = runAStar(extended_adj_matrix, start_idx, goal_idx);

    // Remove the temporary nodes from the nodes vector
    nodes.pop_back();
    nodes.pop_back();

    return path;
}

/// @brief Checks if there is a clear line of sight from p1 to p2.
/// @return true if the line segment from p1 to p2 does not intersect any wall segments, false otherwise
bool PathPlanner::hasLineOfSight(Point2d p1, Point2d p2) {
    // Check if the path from p1 to p2 intersects with any wall segment
    for (size_t i = 0; i < walls.size(); ++i) {
        const auto& wall = walls[i];
        if (doSegmentsIntersect(p1, p2, wall.p1, wall.p2)) {
            return false;  // Path is blocked by this wall
        }
    }
    return true;  // Path is clear
}

/// @brief Runs the A* algorithm to find the shortest path from start_node_idx to goal_node_idx.
/// @param temp_adj_matrix The adjacency matrix representing edge weights between nodes
/// @param start_node_idx Index of the starting node
/// @param goal_node_idx Index of the goal node
/// @return A vector of points representing the shortest path from start to goal
std::vector<Point2d> PathPlanner::runAStar(const std::vector<std::vector<double>>& temp_adj_matrix, int start_node_idx, int goal_node_idx) {
    int num_nodes = nodes.size();

    // Heuristic: Euclidean distance to goal
    auto heuristic = [this, goal_node_idx](int node_idx) -> double {
        return nodes[node_idx].pos.Distance(nodes[goal_node_idx].pos);
    };

    // Data structures for A* algorithm
    std::vector<double> g_score(num_nodes, std::numeric_limits<double>::infinity());
    std::vector<double> f_score(num_nodes, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(num_nodes, -1);
    std::vector<bool> in_open_set(num_nodes, false);
    std::vector<bool> closed_set(num_nodes, false);

    // Priority queue: (f_score, node_idx)
    // We use a custom comparator for min-heap behavior
    auto cmp = [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first > b.first;  // Min-heap (lowest f_score has highest priority)
    };
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> open_set(cmp);

    // Initialize start node
    g_score[start_node_idx] = 0.0;
    f_score[start_node_idx] = heuristic(start_node_idx);
    open_set.push({f_score[start_node_idx], start_node_idx});
    in_open_set[start_node_idx] = true;

    // Main A* loop
    while (!open_set.empty()) {
        // Get node with lowest f_score from open set
        auto [current_f, current_idx] = open_set.top();
        open_set.pop();
        in_open_set[current_idx] = false;

        // Goal found
        if (current_idx == goal_node_idx) {
            // Reconstruct path
            std::vector<Point2d> path;
            int node = goal_node_idx;
            while (node != -1) {
                path.push_back(nodes[node].pos);
                node = came_from[node];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed_set[current_idx] = true;

        // Explore neighbors
        for (int neighbor_idx = 0; neighbor_idx < num_nodes; ++neighbor_idx) {
            // Skip if no edge or neighbor is in closed set
            if (temp_adj_matrix[current_idx][neighbor_idx] < 0 || closed_set[neighbor_idx]) {
                continue;
            }

            double edge_weight = temp_adj_matrix[current_idx][neighbor_idx];
            double tentative_g = g_score[current_idx] + edge_weight;

            // If we found a better path to the neighbor
            if (tentative_g < g_score[neighbor_idx]) {
                came_from[neighbor_idx] = current_idx;
                g_score[neighbor_idx] = tentative_g;
                f_score[neighbor_idx] = g_score[neighbor_idx] + heuristic(neighbor_idx);

                // Add to open set if not already there
                if (!in_open_set[neighbor_idx]) {
                    open_set.push({f_score[neighbor_idx], neighbor_idx});
                    in_open_set[neighbor_idx] = true;
                }
            }
        }
    }

    // No path found, return empty vector
    return std::vector<Point2d>();
}

/// @brief The C-style interface function to get a path from start to goal (findPath under the hood).
/// @param start The starting point
/// @param goal The goal point
/// @param path_buffer The buffer to store the path points
/// @param max_size The maximum size of the path buffer
/// @return The euclidean length of the path found, or -1 if unsuccessful (no path exists or buffer is too small)
double get_path(Point2d start, Point2d goal, Point2d* path_buffer, int max_size) {
    // Create a static instance so it's only initialized once.
    static PathPlanner planner;

    // Call the C++ class method to get the path
    std::vector<Point2d> result_path = planner.findPath(start, goal);

    // Copy the result into the C buffer, checking for size limits
    if ((int)result_path.size() > max_size) {
        // Handle error: buffer is too small
        return -1;
    }

    for (size_t i = 0; i < result_path.size(); ++i) {
        path_buffer[i] = result_path[i];
    }

    // Return the euclidean length of the path found
    double path_length = 0.0;
    for (size_t i = 1; i < result_path.size(); ++i) {
        path_length += result_path[i - 1].Distance(result_path[i]);
    }
    return path_length;
}

/// @brief Checks if a point is inside one of the interior walls (obstacles).
/// @details The interior walls are rectangular obstacles:
///          - Vertical wall BIJC: x ∈ [0.07, 0.18], y ∈ [-0.25, 0.575]
///          - Horizontal wall EFHG: x ∈ [-0.575, -0.215], y ∈ [-0.055, 0.055]
/// @param point The point to check
/// @return true if the point is inside either wall rectangle, false otherwise
bool is_point_in_interior_wall(Point2d point) {
    // Vertical wall BIJC
    // B(0.07, 0.575), I(0.07, -0.25), J(0.18, -0.25), C(0.18, 0.575)
    // x range: [0.07, 0.18], y range: [-0.25, 0.575]
    const double BIJC_X_MIN = 0.07;
    const double BIJC_X_MAX = 0.18;
    const double BIJC_Y_MIN = -0.25;
    const double BIJC_Y_MAX = 0.575;

    bool in_bijc = (point.x >= BIJC_X_MIN && point.x <= BIJC_X_MAX && point.y >= BIJC_Y_MIN && point.y <= BIJC_Y_MAX);

    // Horizontal wall EFHG
    // E(-0.575, 0.055), F(-0.215, 0.055), H(-0.215, -0.055), G(-0.575, -0.055)
    // x range: [-0.575, -0.215], y range: [-0.055, 0.055]
    const double EFHG_X_MIN = -0.575;
    const double EFHG_X_MAX = -0.215;
    const double EFHG_Y_MIN = -0.055;
    const double EFHG_Y_MAX = 0.055;

    bool in_efhg = (point.x >= EFHG_X_MIN && point.x <= EFHG_X_MAX && point.y >= EFHG_Y_MIN && point.y <= EFHG_Y_MAX);

    return in_bijc || in_efhg;
}