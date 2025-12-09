#ifndef TV_LINEAR_SHORTEST_PATH_H
#define TV_LINEAR_SHORTEST_PATH_H

#include "geometry.h"
#include <vector>

// Implements the Linear Time O(N) Funnel/String-Pulling algorithm.
// Computes the 'taut string' visibility path inside P.
// This identifies the exact topological constraints (Reflex Vertices) required 
// to build the Splinegon Diagram boundaries.
class LinearShortestPath {
    const Polygon& P;

public:
    explicit LinearShortestPath(const Polygon& poly) : P(poly) {}

    // Computes path from Start -> End using Deque-based monotonic chain reduction.
    // Complexity: Strict O(n) (Each vertex pushed/popped at most twice).
    std::vector<Point> compute(Point start, Point end) const;
};

#endif // TV_LINEAR_SHORTEST_PATH_H