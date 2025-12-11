#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>
#include <iostream>

// Represents a 2D point in the arena
struct Point2d {
    double x;
    double y;

    Point2d(double xin = 0, double yin = 0) : x(xin), y(yin) {}

    // Operator overloads for easier math
    bool operator==(const Point2d& other) const {
        return (std::abs(x - other.x) < 1e-9 && std::abs(y - other.y) < 1e-9);
    }

    // Euclidean distance
    double distanceTo(const Point2d& p) const {
        return std::sqrt(std::pow(x - p.x, 2) + std::pow(y - p.y, 2));
    }

    // Distance from this point to a line segment defined by a and b
    double distanceToSegment(const Point2d& a, const Point2d& b) const {
        double l2 = std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2);
        if (l2 == 0.0) return distanceTo(a);

        double t = ((x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y)) / l2;
        if (t < 0.0) return distanceTo(a);
        if (t > 1.0) return distanceTo(b);

        Point2d projection(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
        return distanceTo(projection);
    }
};

// Represents a wall segment for physics/collision checks
struct LineSegment {
    Point2d p1;
    Point2d p2;
    LineSegment(const Point2d& point1, const Point2d& point2) : p1(point1), p2(point2) {}
};

// Represents a node in the visibility graph
struct GraphNode {
    char id;
    Point2d pos;
};

// --- Geometric Helper Functions ---

// Returns orientation of ordered triplet (p, q, r).
// 0 -> collinear, 1 -> clockwise, 2 -> counterclockwise
static int orientation(Point2d p, Point2d q, Point2d r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (std::abs(val) < 1e-9) return 0;
    return (val > 0) ? 1 : 2;
}

// Checks if point q lies on line segment 'pr'
static bool onSegment(Point2d p, Point2d q, Point2d r) {
    return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) &&
            q.y >= std::min(p.y, r.y));
}

// Returns true if line segments 'p1q1' and 'p2q2' intersect
static bool doSegmentsIntersect(Point2d p1, Point2d q1, Point2d p2, Point2d q2) {
    const double eps = 1e-9;
    auto pointsEqual = [eps](Point2d a, Point2d b) {
        return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps;
    };

    if (pointsEqual(p1, p2) || pointsEqual(p1, q2) || pointsEqual(q1, p2) || pointsEqual(q1, q2))
        return false;

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0 && o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(p1, p2, q1) && !pointsEqual(p2, p1) && !pointsEqual(p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1) && !pointsEqual(q2, p1) && !pointsEqual(q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2) && !pointsEqual(p1, p2) && !pointsEqual(p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2) && !pointsEqual(q1, p2) && !pointsEqual(q1, q2)) return true;

    return false;
}

#endif  // GEOMETRY_HPP