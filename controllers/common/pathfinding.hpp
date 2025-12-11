#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include <vector>

#include "geometry.hpp"

struct Path {
    std::vector<Point2d> waypoints;
    double totalDistance = 0.0;
};

class PathPlanner {
   public:
    PathPlanner();

    /**
     * @brief Finds shortest path from start to goal using A* on visibility graph.
     * @return Vector of points (start -> ... -> goal). Empty if no path.
     */
    Path findPath(const Point2d& start, const Point2d& goal);

    /**
     * @brief Public access to adjacency matrix (useful for debug visualization)
     */
    const std::vector<std::vector<double>>& getAdjMatrix() const { return adjMatrix_; }

   private:
    // The adjacency matrix is precomputed based on MapConfig
    std::vector<std::vector<double>> adjMatrix_;
};

#endif  // PATHFINDING_HPP