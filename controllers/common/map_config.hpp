#ifndef MAP_CONFIG_HPP
#define MAP_CONFIG_HPP

#include <vector>

#include "geometry.hpp"

namespace MapConfig {

// --- Graph Nodes (Visibility Graph) ---
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

static const std::vector<GraphNode> nodes = {
    {'A', {-0.58, 0.58}},    // 0
    {'B', {0.075, 0.58}},    // 1
    {'C', {0.175, 0.58}},    // 2
    {'D', {0.58, 0.58}},     // 3
    {'E', {-0.58, 0.05}},    // 4
    {'F', {-0.215, 0.05}},   // 5
    {'G', {-0.58, -0.05}},   // 6
    {'H', {-0.215, -0.05}},  // 7
    {'I', {0.075, -0.25}},   // 8
    {'J', {0.175, -0.25}},   // 9
    {'K', {-0.58, -0.58}},   // 10
    {'L', {0.58, -0.58}}     // 11
};

// --- Wall Definitions ---
// Includes outer perimeter, interior obstacles, and centerline blockers
static const std::vector<LineSegment> walls = {
    // Outer boundary
    {{nodes[0].pos}, {nodes[3].pos}},    // Top
    {{nodes[0].pos}, {nodes[10].pos}},   // Left
    {{nodes[10].pos}, {nodes[11].pos}},  // Bottom
    {{nodes[3].pos}, {nodes[11].pos}},   // Right

    // Vertical Obstacle (BIJC)
    {{nodes[1].pos}, {nodes[8].pos}},  // Left
    {{nodes[8].pos}, {nodes[9].pos}},  // Bottom
    {{nodes[9].pos}, {nodes[2].pos}},  // Right
    // Centerline blocker for BIJC
    {{(nodes[1].pos.x + nodes[2].pos.x) / 2.0, (nodes[1].pos.y + nodes[2].pos.y) / 2.0 + 0.01},
     {(nodes[8].pos.x + nodes[9].pos.x) / 2.0, (nodes[8].pos.y + nodes[9].pos.y) / 2.0 + 0.01}},

    // Horizontal Obstacle (EFHG)
    {{nodes[4].pos}, {nodes[5].pos}},  // Top
    {{nodes[5].pos}, {nodes[7].pos}},  // Right
    {{nodes[7].pos}, {nodes[6].pos}},  // Bottom
    // Centerline blocker for EFHG
    {{(nodes[4].pos.x + nodes[6].pos.x) / 2.0 - 0.01, (nodes[4].pos.y + nodes[6].pos.y) / 2.0},
     {(nodes[5].pos.x + nodes[7].pos.x) / 2.0 - 0.01, (nodes[5].pos.y + nodes[7].pos.y) / 2.0}}};

// --- Utility: Check if point is inside an obstacle ---
inline bool isPointInInteriorWall(const Point2d& p, double padding = 0.0) {
    // Vertical wall BIJC bounds
    const double BIJC_X_MIN = nodes[1].pos.x - padding;  // Use B.x
    const double BIJC_X_MAX = nodes[2].pos.x + padding;  // Use C.x
    const double BIJC_Y_MIN = nodes[8].pos.y - padding;  // Use J.y
    const double BIJC_Y_MAX = nodes[1].pos.y + padding;  // Use B.y

    bool in_bijc = (p.x >= BIJC_X_MIN && p.x <= BIJC_X_MAX && p.y >= BIJC_Y_MIN && p.y <= BIJC_Y_MAX);

    // Horizontal wall EFHG bounds
    const double EFHG_X_MIN = nodes[4].pos.x - padding;  // Use E.x
    const double EFHG_X_MAX = nodes[5].pos.x + padding;  // Use F.x
    const double EFHG_Y_MIN = nodes[7].pos.y - padding;  // Use H.y
    const double EFHG_Y_MAX = nodes[5].pos.y + padding;  // Use F.y

    bool in_efhg = (p.x >= EFHG_X_MIN && p.x <= EFHG_X_MAX && p.y >= EFHG_Y_MIN && p.y <= EFHG_Y_MAX);

    return in_bijc || in_efhg;
}

// Checks if there is a direct line of sight between points p1 and p2 (no walls in between)
inline bool hasLineOfSight(const Point2d& p1, const Point2d& p2) {
    for (const auto& wall : walls) {
        if (doSegmentsIntersect(p1, p2, wall.p1, wall.p2)) return false;
    }
    return true;
}

}  // namespace MapConfig

#endif  // MAP_CONFIG_HPP