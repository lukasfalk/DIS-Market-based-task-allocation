#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include <vector>

#include "geometry.hpp"

class PathPlanner {
   public:
    PathPlanner();

    /**
     * @brief Finds shortest path from start to goal using A* on visibility graph.
     * @return Vector of points (start -> ... -> goal). Empty if no path.
     */
    std::vector<Point2d> findPath(const Point2d& start, const Point2d& goal);

    /**
     * @brief Public access to adjacency matrix (useful for debug visualization)
     */
    const std::vector<std::vector<double>>& getAdjMatrix() const { return adj_matrix_; }

   private:
    // The adjacency matrix is precomputed based on MapConfig
    std::vector<std::vector<double>> adj_matrix_;
};

/**
 * @brief C-style interface for pathfinding (compatibility wrapper)
 * @param start Start point
 * @param goal Goal point
 * @param path_buffer Buffer to store the path
 * @param max_size Maximum number of points in buffer
 * @return Total path length, or -1.0 if failed
 */
double get_path(Point2d start, Point2d goal, Point2d* path_buffer, int max_size);

#endif  // PATHFINDING_HPP