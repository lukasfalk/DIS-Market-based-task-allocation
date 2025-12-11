#include "pathfinding.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>

#include "geometry.hpp"
#include "map_config.hpp"

// --- PathPlanner Implementation ---

PathPlanner::PathPlanner() {
    size_t num_nodes = MapConfig::nodes.size();
    adj_matrix_.assign(num_nodes, std::vector<double>(num_nodes, -1.0));

    for (size_t i = 0; i < num_nodes; ++i) {
        for (size_t j = i; j < num_nodes; ++j) {
            if (i == j) {
                adj_matrix_[i][j] = 0;
                continue;
            }
            if (MapConfig::hasLineOfSight(MapConfig::nodes[i].pos, MapConfig::nodes[j].pos)) {
                double dist = MapConfig::nodes[i].pos.distance(MapConfig::nodes[j].pos);
                adj_matrix_[i][j] = dist;
                adj_matrix_[j][i] = dist;
            }
        }
    }
}

std::vector<Point2d> PathPlanner::findPath(const Point2d& start, const Point2d& goal) {
    // 1. Setup Graph with Start/Goal nodes
    size_t num_static = MapConfig::nodes.size();
    int start_idx = (int)num_static;
    int goal_idx = (int)num_static + 1;
    size_t total_nodes = num_static + 2;

    std::vector<std::vector<double>> temp_adj = adj_matrix_;
    for (auto& row : temp_adj) {
        row.resize(total_nodes, -1.0);
    }
    temp_adj.resize(total_nodes, std::vector<double>(total_nodes, -1.0));

    temp_adj[start_idx][start_idx] = 0;
    temp_adj[goal_idx][goal_idx] = 0;

    auto connect = [&](int node_idx, Point2d pos) {
        for (size_t i = 0; i < num_static; ++i) {
            if (MapConfig::hasLineOfSight(pos, MapConfig::nodes[i].pos)) {
                double d = pos.distance(MapConfig::nodes[i].pos);
                temp_adj[node_idx][i] = d;
                temp_adj[i][node_idx] = d;
            }
        }
    };

    connect(start_idx, start);
    connect(goal_idx, goal);

    if (MapConfig::hasLineOfSight(start, goal)) {
        double d = start.distance(goal);
        temp_adj[start_idx][goal_idx] = d;
        temp_adj[goal_idx][start_idx] = d;
    }

    // 2. A* Algorithm (Inlined)
    auto getPos = [&](int idx) {
        if (idx < (int)num_static) return MapConfig::nodes[idx].pos;
        return (idx == start_idx) ? start : goal;
    };

    std::vector<double> g_score(total_nodes, std::numeric_limits<double>::infinity());
    std::vector<double> f_score(total_nodes, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(total_nodes, -1);
    std::vector<bool> in_open_set(total_nodes, false);

    // Min-heap
    using PItem = std::pair<double, int>;
    std::priority_queue<PItem, std::vector<PItem>, std::greater<PItem>> open_set;

    g_score[start_idx] = 0.0;
    f_score[start_idx] = start.distance(goal);
    open_set.push({f_score[start_idx], start_idx});
    in_open_set[start_idx] = true;

    while (!open_set.empty()) {
        auto [current_f, current_idx] = open_set.top();
        open_set.pop();
        in_open_set[current_idx] = false;

        if (current_idx == goal_idx) {
            // Reconstruct
            std::vector<Point2d> path;
            int curr = goal_idx;
            while (curr != -1) {
                path.push_back(getPos(curr));
                curr = came_from[curr];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < (int)total_nodes; ++neighbor) {
            double weight = temp_adj[current_idx][neighbor];
            if (weight < 0) continue;  // no edge

            double tentative_g = g_score[current_idx] + weight;
            if (tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current_idx;
                g_score[neighbor] = tentative_g;
                f_score[neighbor] = tentative_g + getPos(neighbor).distance(goal);

                if (!in_open_set[neighbor]) {
                    open_set.push({f_score[neighbor], neighbor});
                    in_open_set[neighbor] = true;
                }
            }
        }
    }

    return {};  // No path
}

double get_path(Point2d start, Point2d goal, Point2d* path_buffer, int max_size) {
    static PathPlanner planner;
    std::vector<Point2d> path = planner.findPath(start, goal);

    if (path.empty()) return -1.0;
    if ((int)path.size() > max_size) return -1.0;

    double total_dist = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
        path_buffer[i] = path[i];
        if (i > 0) {
            total_dist += path[i].distance(path[i - 1]);
        }
    }

    // Fill remaining buffer with 0 (optional, but good for safety)
    if ((int)path.size() < max_size) {
        // Mark end with 0,0 if needed, or just rely on return value?
        // The caller seems to check for 0,0 in epuck_crown.cpp:
        // while (count < MAX_PATH_LENGTH && (waypoint_path[count].x != 0 || waypoint_path[count].y != 0))
        // So we should zero out the rest.
        for (int i = path.size(); i < max_size; ++i) {
            path_buffer[i] = Point2d(0, 0);
        }
    }

    return total_dist;
}
