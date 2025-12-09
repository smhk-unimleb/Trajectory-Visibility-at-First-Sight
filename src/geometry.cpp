#include "geometry.h"

Segment Polygon::get_edge(const size_t& i) const {
    return Segment{ get_vertex(i), get_vertex(i + 1) };
}

// Consistent naming: cross_product_z
double cross_product_z(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool Polygon::is_reflex(size_t i) const {
    size_t n = vertices.size();
    Point prev = vertices[(i + n - 1) % n];
    Point curr = vertices[i];
    Point next = vertices[(i + 1) % n];

    // Assumes CCW orientation. Right turn (negative cross product) = Reflex.
    return cross_product_z(prev, curr, next) < -EPSILON;
}

double dist_sq(Point a, Point b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

// Orientation Helper:
// 0: Collinear, 1: CW, 2: CCW
int orientation(Point p, Point q, Point r) {
    double val = cross_product_z(p, q, r);
    if (std::abs(val) < EPSILON) return 0;
    return (val < 0) ? 1 : 2; // Neg: CW(1), Pos: CCW(2)
}

bool on_segment(const Point& p, const Segment &s) {
    return p.x <= std::max(s.p1.x, s.p2.x) && p.x >= std::min(s.p1.x, s.p2.x) &&
           p.y <= std::max(s.p1.y, s.p2.y) && p.y >= std::min(s.p1.y, s.p2.y);
}

bool segments_intersect(Segment s1, Segment s2) {
    Point p1 = s1.p1, q1 = s1.p2;
    Point p2 = s2.p1, q2 = s2.p2;

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && on_segment(p2, s1)) return true;
    if (o2 == 0 && on_segment(q2, s1)) return true;
    if (o3 == 0 && on_segment(p1, s2)) return true;
    if (o4 == 0 && on_segment(q1, s2)) return true;
    return false;
}

bool is_point_in_polygon(const Polygon& P, const Point& p) {
    bool inside = false;
    size_t n = P.size();
    for (size_t i = 0; i < n; i++) {
        Point v1 = P.get_vertex(i);
        Point v2 = P.get_vertex(i + 1);

        bool intersects_y = ((v1.y > p.y) != (v2.y > p.y));
        if (intersects_y) {
            double x_inters = (v2.x - v1.x) * (p.y - v1.y) / (v2.y - v1.y) + v1.x;
            if (p.x < x_inters) {
                inside = !inside;
            }
        }
        if (on_segment(p, {v1, v2})) return true;
    }
    return inside;
}

bool is_visible_naive(const Polygon& P, Point q, Point r) {
    if (q == r) return is_point_in_polygon(P, q);
    Segment query_seg = {q, r};

    for (size_t i = 0; i < P.size(); ++i) {
        Segment edge = P.get_edge(i);

        // Simple bounding box/vertex rejection omitted for "Strict" Logic
        if (segments_intersect(query_seg, edge)) {
             // Handle endpoints
            bool touching = (q == edge.p1 || q == edge.p2 || r == edge.p1 || r == edge.p2);
            if (touching) continue;

            // Allow grazing exactly on vertex (handled by on_segment checks below)
            if (on_segment(edge.p1, query_seg) || on_segment(edge.p2, query_seg)) continue;

            // Strict Crossing
            int o1 = orientation(query_seg.p1, query_seg.p2, edge.p1);
            int o2 = orientation(query_seg.p1, query_seg.p2, edge.p2);
            int o3 = orientation(edge.p1, edge.p2, query_seg.p1);
            int o4 = orientation(edge.p1, edge.p2, query_seg.p2);

            if (o1 != o2 && o3 != o4) return false;
        }
    }
    return true;
}