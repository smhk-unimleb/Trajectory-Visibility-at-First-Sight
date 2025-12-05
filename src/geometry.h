#ifndef TV_GEOMETRY_H
#define TV_GEOMETRY_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

constexpr double EPSILON = 1e-9;

struct Point {
    double x;
    double y;

    bool operator==(const Point& other) const {
        return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
    }
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
    Point operator+(const Point& other) const { return {x + other.x, y + other.y}; }
    Point operator-(const Point& other) const { return {x - other.x, y - other.y}; }
    Point operator*(double scalar) const { return {x * scalar, y * scalar}; }
    Point operator/(double scalar) const { return {x / scalar, y / scalar}; }
};

using Vector2D = Point;

struct Segment {
    Point p1;
    Point p2;
};

struct Polygon {
    std::vector<Point> vertices;

    void add_vertex(double x, double y) {
        vertices.push_back({x, y});
    }

    size_t size() const {
        return vertices.size();
    }

    Segment get_edge(const size_t &i) const;
    Point get_vertex(size_t i) const;

    // Checks if vertex i is reflex (interior angle > 180 degrees)
    // Assumes CCW winding order.
    bool is_reflex(size_t i) const;
};

// --- Primitives ---

// Cross product (z-component)
double cross_product_z(Point a, Point b, Point c);
double dist_sq(Point a, Point b);

// Intersection tests
bool on_segment(const Point& p, const Segment &s);
bool segments_intersect(Segment s1, Segment s2);

// Polygon inclusion and visibility
bool is_point_in_polygon(const Polygon& P, const Point& p);

// Checks if segment qr intersects any edge of P strictly
bool is_visible_naive(const Polygon& P, Point q, Point r);

#endif // TV_GEOMETRY_H