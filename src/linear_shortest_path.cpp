#include "linear_shortest_path.h"
#include <deque>
#include <iostream>
#include <vector>
#include <algorithm>

// Bring in visibility check
#include "geometry.h"

LinearShortestPath::LinearShortestPath(const Polygon& poly) : P(poly) {
    double area = 0.0;
    const size_t n = P.size();
    const Point origin{0,0};

    for(size_t i=0; i<n; ++i) {
        area += cross_product_z(origin, P.get_vertex(i), P.get_vertex(i+1));
    }
    ccw_winding = (area > EPSILON);
}

// Logic: > 0 for Left Turn, < 0 for Right Turn.
static inline double turn_val(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

std::vector<Point> LinearShortestPath::compute(Point start, Point end) const {
    if (start == end) return {start};

    // OPTIMIZATION for Theorem 1 O(N):
    // If the path is strictly unobstructed, it is the straight line.
    // This resolves issues where "Hull" logic indiscriminately wraps disjoint reflex vertices.
    if (is_visible_naive(P, start, end)) {
        return {start, end};
    }

    // Fallback: Path is obstructed. Run Monotone Chain Hull on Constraints.
    // 1. Gather Candidate Pivots
    std::vector<Point> pivots;
    pivots.reserve(P.size() + 2);
    pivots.push_back(start);

    for(size_t i=0; i<P.size(); ++i) {
        if (P.is_reflex(i)) {
            Point v = P.get_vertex(i);
            if (v != start && v != end) {
                pivots.push_back(v);
            }
        }
    }
    pivots.push_back(end);

    std::deque<Point> path;
    path.push_back(start);

    for (const Point& v : pivots) {
        if (v == start) continue;

        while (path.size() >= 2) {
            Point p1 = path[path.size() - 2];
            Point p2 = path.back();

            if (turn_val(p1, p2, v) > EPSILON) {
                 path.pop_back();
            } else {
                 break;
            }
        }
        path.push_back(v);
    }

    std::vector<Point> res;
    res.reserve(path.size());
    for(auto p : path) res.push_back(p);

    if (!res.empty() && res.back() != end) res.push_back(end);

    const auto last = ranges::unique(res).begin();
    res.erase(last, res.end());

    return res;
}