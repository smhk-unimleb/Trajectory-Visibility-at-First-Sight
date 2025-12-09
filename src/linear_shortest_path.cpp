#include "linear_shortest_path.h"
#include <deque>
#include <iostream>
#include <cmath>
#include <limits>

LinearShortestPath::LinearShortestPath(const Polygon& poly) : P(poly) {
    // 1. Determine strict winding (Signed Area).
    double area = 0.0;
    size_t n = P.size();
    Point origin{0,0};

    for(size_t i=0; i<n; ++i) {
        area += cross_product_z(origin, P.get_vertex(i), P.get_vertex(i+1));
    }

    // If area > 0, it is CCW.
    ccw_winding = (area > EPSILON);
}

std::vector<Point> LinearShortestPath::compute(Point start, Point end) const {
    if (start == end)
      return {start};

    // The "Funnel" / Monotone Chain Path Logic.
    std::deque<Point> path;
    path.push_back(start);

    // Scan P vertices
    for (size_t i = 0; i < P.size(); ++i) {
        Point v = P.get_vertex(i);

        // Skip endpoints if they match polygon vertices to avoid dupes
        if (v == start || v == end)
          continue;

        while (path.size() >= 2) {
            Point p1 = path[path.size() - 2];
            Point p2 = path.back();

            // Turn Check
            double turn = cross_product_z(p1, p2, v); // FIXED: Used correct function name

            // Winding Logic:
            // CCW Polygon: Interior is Left. Convex Vertex makes Left Turn (>0). Reflex makes Right Turn (<0).
            // CW Polygon: Interior is Right. Convex Vertex makes Right Turn (<0). Reflex makes Left Turn (>0).

            bool is_convex_vertex = false;

            if (ccw_winding) {
                // If turn > 0 (Left), the vertex is convex (outward/loose).
                is_convex_vertex = (turn > EPSILON);
            } else {
                // If turn < 0 (Right), the vertex is convex (outward/loose) relative to CW winding.
                is_convex_vertex = (turn < -EPSILON);
            }

            if (is_convex_vertex) {
                // This vertex makes the string loose. Remove the pivot p2.
                path.pop_back();
            } else {
                // Vertex makes string tight (Reflex or Collinear).
                // It is a valid pivot (or creates a tighter winding). Break and push.
                break;
            }
        }
        path.push_back(v);
    }

    path.push_back(end);

    std::vector<Point> res;
    for(auto p : path) {
        if (res.empty() || res.back() != p)
          res.push_back(p);
    }
    return res;
}