
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "../auct_super/Point2d.h"

// Struct for nodes of the visibility graph
struct GraphNode {
    char id;
    Point2d pos;
};

// Represents a wall segment for line-of-sight checks
struct LineSegment {
    Point2d p1;
    Point2d p2;

    // Constructor for initialization
    LineSegment(const Point2d& point1, const Point2d& point2) : p1(point1), p2(point2) {}
};

class PathPlanner {
   public:
    PathPlanner();

    // The main public function to find a path
    std::vector<Point2d> findPath(Point2d start, Point2d goal);

    // Public visualization method (for supervisor to call)
    // This method is public so the supervisor can visualize the graph
    const std::vector<GraphNode>& getNodes() const { return nodes; }
    const std::vector<std::vector<double>>& getAdjMatrix() const { return adj_matrix; }

   private:
    // --- Private Member Variables ---

    // A list of all the static nodes in our visibility graph
    std::vector<GraphNode> nodes;

    // A list of all wall segments for line intersection checks
    std::vector<LineSegment> walls;

    // Adjacency matrix to store graph connections and weights (distances)
    // adj_matrix[i][j] > 0 means there is an edge from node i to j with that weight
    // adj_matrix[i][j] = 0 means node i == node j
    // adj_matrix[i][j] = -1.0 means no direct edge
    std::vector<std::vector<double>> adj_matrix;

    // --- Private Helper Functions ---

    // Checks if there is a clear line of sight between two points
    bool hasLineOfSight(Point2d p1, Point2d p2);

    // The actual A* algorithm implementation
    std::vector<Point2d> runAStar(const std::vector<std::vector<double>>& temp_adj_matrix, int start_node_idx, int goal_node_idx);
};

// --- C Interface for the e-puck C controller ---
#ifdef __cplusplus
extern "C" {
#endif

double get_path(Point2d start, Point2d goal, Point2d* path_buffer, int max_size);

/// @brief Checks if a point is inside one of the interior walls (obstacles).
/// @param point The point to check
/// @return true if the point is inside either the vertical wall (BIJC) or horizontal wall (EFHG), false otherwise
bool is_point_in_interior_wall(Point2d point);

#ifdef __cplusplus
}
#endif

#endif  // PATH_PLANNER_H