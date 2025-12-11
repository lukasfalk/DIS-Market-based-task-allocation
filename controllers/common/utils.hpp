#ifndef UTILS_HPP
#define UTILS_HPP

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace utils {

// Generate random number in [0,1]
inline double random_01() { return ((double)rand()) / ((double)RAND_MAX); }

// Limit a value (clamp)
template <typename T>
inline void limit(T* number, T limit_val) {
    if (*number > limit_val) *number = limit_val;
    if (*number < -limit_val) *number = -limit_val;
}

// Euclidean distance between two points (x0, y0) and (x1, y1)
inline double dist(double x0, double y0, double x1, double y1) {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

}  // namespace utils

#endif  // UTILS_HPP
