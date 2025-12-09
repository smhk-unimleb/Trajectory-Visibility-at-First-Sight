#include "geometry.h"

Segment Polygon::get_edge(const size_t& i) const {
    return Segment{ get_vertex(i), get_vertex(i + 1) };
}

double cross_product_z(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// CCW Winding: Interior Left => Reflex Right Turn (< 0)
bool Polygon::is_reflex(size_t i) const {
    const Point prev = get_vertex(i + vertices.size() - 1), curr = get_vertex(i),
            next = get_vertex(i + 1);
    return cross_product_z(prev, curr, next) < -EPSILON;
}

double dist_sq(const Point a, const Point b) {
    const double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

int orientation(const Point p, const Point q, const Point r) {
    const double val = cross_product_z(p, q, r);
    if (std::abs(val) < EPSILON) return 0;
    return (val < 0) ? 1 : 2;
}

bool on_segment(const Point& p, const Segment &s) {
    return p.x <= std::max(s.p1.x, s.p2.x) && p.x >= std::min(s.p1.x, s.p2.x) &&
           p.y <= std::max(s.p1.y, s.p2.y) && p.y >= std::min(s.p1.y, s.p2.y);
}

bool segments_intersect(const Segment &s1, const Segment &s2) {
    const Point p1 = s1.p1, q1 = s1.p2, p2 = s2.p1, q2 = s2.p2;
    const int o1 = orientation(p1, q1, p2), o2 = orientation(p1, q1, q2),
        o3 = orientation(p2, q2, p1), o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;
    if (o1 == 0 && on_segment(p2, s1))
        return true;
    if (o2 == 0 && on_segment(q2, s1))
        return true;
    if (o3 == 0 && on_segment(p1, s2))
        return true;
    if (o4 == 0 && on_segment(q1, s2))
        return true;
    return false;
}

bool is_point_in_polygon(const Polygon& P, const Point& p) {
    bool inside = false;
    for (size_t i = 0; i < P.size(); i++) {
        const Point v1 = P.get_vertex(i), v2 = P.get_vertex(i + 1);
        if ((v1.y > p.y) != (v2.y > p.y)) {
            if (p.x < ((v2.x - v1.x) * (p.y - v1.y) / (v2.y - v1.y) + v1.x))
                inside = !inside;
        }
        if (on_segment(p, {v1, v2}))
            return true; // Boundary inclusion
    }
    return inside;
}

bool is_visible_naive(const Polygon& P, Point q, Point r) {
    if (q == r) return is_point_in_polygon(P, q);

    // A segment connected by two points must be inside the polygon.
    // If not, q and r might be "visible" via the empty space outside P (e.g. crossing ceiling).
    if (!is_point_in_polygon(P, (q + r) / 2.0))
        return false;

    const Segment query_seg = {q, r};
    for (size_t i = 0; i < P.size(); ++i) {
        Segment edge = P.get_edge(i);
        if (segments_intersect(query_seg, edge)) {
            // Endpoints touch
            if ((q == edge.p1 || q == edge.p2 || r == edge.p1 || r == edge.p2))
                continue;
            // Grazing boundary lines (collinear overlap with edge) -> treated as visible if strictly on it
            if (on_segment(edge.p1, query_seg) || on_segment(edge.p2, query_seg))
                continue;

            const int o1 = orientation(query_seg.p1, query_seg.p2, edge.p1),
                o2 = orientation(query_seg.p1, query_seg.p2, edge.p2),
                o3 = orientation(edge.p1, edge.p2, query_seg.p1),
                o4 = orientation(edge.p1, edge.p2, query_seg.p2);

            if (o1 != o2 && o3 != o4)
                return false;
        }
    }
    return true;
}