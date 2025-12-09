#ifndef TV_LINEAR_SHORTEST_PATH_H
#define TV_LINEAR_SHORTEST_PATH_H

#include "geometry.h"
#include <vector>
using namespace std;

// Implements a refined O(N) Shortest Path logic by acting
// exclusively on Polygon Reflex Vertices (Geometric constraints).
class LinearShortestPath {
    const Polygon& P;
    bool ccw_winding;

public:
    explicit LinearShortestPath(const Polygon& poly);

    // Computes topological shortest path pivot points strictly O(N).
    vector<Point> compute(Point start, Point end) const;
};

#endif // TV_LINEAR_SHORTEST_PATH_H