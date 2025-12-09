#ifndef TV_GEOMETRY_H
#define TV_GEOMETRY_H

#include <vector>
#include <cmath>
#include <iostream>

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
    Point operator*(const double scalar) const { return {x * scalar, y * scalar}; }
    Point operator/(const double scalar) const { return {x / scalar, y / scalar}; }
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

    // Safely get vertex with index modulo size
    Point get_vertex(size_t i) const {
        return vertices[i % vertices.size()];
    }

    Segment get_edge(const size_t &i) const;
    bool is_reflex(size_t i) const;
};

// --- Primitives ---

// Cross product z-component: (b.x-a.x)(c.y-a.y) - (b.y-a.y)(c.x-a.x)
// > 0 if a->b->c is Counter-Clockwise (Left Turn)
double cross_product_z(Point a, Point b, Point c);

double dist_sq(Point a, Point b);

// Intersection tests
bool on_segment(const Point& p, const Segment &s);
bool segments_intersect(Segment s1, Segment s2);

// Polygon inclusion and visibility
bool is_point_in_polygon(const Polygon& P, const Point& p);
bool is_visible_naive(const Polygon& P, Point q, Point r);

#endif // TV_GEOMETRY_H