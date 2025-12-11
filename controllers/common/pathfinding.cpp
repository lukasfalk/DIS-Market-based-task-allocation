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
    size_t numNodes = MapConfig::nodes.size();
    adjMatrix_.assign(numNodes, std::vector<double>(numNodes, -1.0));

    for (size_t i = 0; i < numNodes; ++i) {
        for (size_t j = i; j < numNodes; ++j) {
            if (i == j) {
                adjMatrix_[i][j] = 0;
                continue;
            }
            if (MapConfig::hasLineOfSight(MapConfig::nodes[i].pos, MapConfig::nodes[j].pos)) {
                double dist = MapConfig::nodes[i].pos.distanceTo(MapConfig::nodes[j].pos);
                adjMatrix_[i][j] = dist;
                adjMatrix_[j][i] = dist;
            }
        }
    }
}

Path PathPlanner::findPath(const Point2d& start, const Point2d& goal) {
    // 1. Setup Graph with Start/Goal nodes
    size_t numStatic = MapConfig::nodes.size();
    int startIdx = (int)numStatic;
    int goalIdx = (int)numStatic + 1;
    size_t totalNodes = numStatic + 2;

    std::vector<std::vector<double>> tempAdj = adjMatrix_;
    for (auto& row : tempAdj) {
        row.resize(totalNodes, -1.0);
    }
    tempAdj.resize(totalNodes, std::vector<double>(totalNodes, -1.0));

    tempAdj[startIdx][startIdx] = 0;
    tempAdj[goalIdx][goalIdx] = 0;

    auto connect = [&](int node_idx, Point2d pos) {
        for (size_t i = 0; i < numStatic; ++i) {
            if (MapConfig::hasLineOfSight(pos, MapConfig::nodes[i].pos)) {
                double d = pos.distanceTo(MapConfig::nodes[i].pos);
                tempAdj[node_idx][i] = d;
                tempAdj[i][node_idx] = d;
            }
        }
    };

    connect(startIdx, start);
    connect(goalIdx, goal);

    if (MapConfig::hasLineOfSight(start, goal)) {
        double d = start.distanceTo(goal);
        tempAdj[startIdx][goalIdx] = d;
        tempAdj[goalIdx][startIdx] = d;
    }

    // 2. A* Algorithm (Inlined)
    auto getPos = [&](int idx) {
        if (idx < (int)numStatic) return MapConfig::nodes[idx].pos;
        return (idx == startIdx) ? start : goal;
    };

    std::vector<double> gCost(totalNodes, std::numeric_limits<double>::infinity());
    std::vector<double> fCost(totalNodes, std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(totalNodes, -1);
    std::vector<bool> inOpenSet(totalNodes, false);

    // Min-heap
    using PItem = std::pair<double, int>;
    std::priority_queue<PItem, std::vector<PItem>, std::greater<PItem>> openSet;

    gCost[startIdx] = 0.0;
    fCost[startIdx] = start.distanceTo(goal);
    openSet.push({fCost[startIdx], startIdx});
    inOpenSet[startIdx] = true;

    while (!openSet.empty()) {
        auto [currentFCost, currentIdx] = openSet.top();
        openSet.pop();
        inOpenSet[currentIdx] = false;

        if (currentIdx == goalIdx) {
            // Reconstruct
            std::vector<Point2d> path;
            int curr = goalIdx;
            while (curr != -1) {
                path.push_back(getPos(curr));
                curr = cameFrom[curr];
            }
            std::reverse(path.begin(), path.end());
            return Path{path, gCost[goalIdx]};
        }

        for (int neighbor = 0; neighbor < (int)totalNodes; ++neighbor) {
            double weight = tempAdj[currentIdx][neighbor];
            if (weight < 0) continue;  // no edge

            double tentativeGCost = gCost[currentIdx] + weight;
            if (tentativeGCost < gCost[neighbor]) {
                cameFrom[neighbor] = currentIdx;
                gCost[neighbor] = tentativeGCost;
                fCost[neighbor] = tentativeGCost + getPos(neighbor).distanceTo(goal);

                if (!inOpenSet[neighbor]) {
                    openSet.push({fCost[neighbor], neighbor});
                    inOpenSet[neighbor] = true;
                }
            }
        }
    }

    // No path
    return Path();
}
