#ifndef TV_LINEAR_SHORTEST_PATH_H
#define TV_LINEAR_SHORTEST_PATH_H

#include "geometry.h"
#include <vector>

class LinearShortestPath {
    const Polygon& P;
    bool ccw_winding; // Stored winding order

public:
    explicit LinearShortestPath(const Polygon& poly);

    // Computes the path using the "Rubberband Algorithm" (String Pulling).
    // It extracts the polygon boundary chain between start/end and relaxes it
    // against the reflex vertices.
    // Complexity: O(n).
    std::vector<Point> compute(Point start, Point end) const;
};

#endif