#ifndef TV_SPLINEGON_H
#define TV_SPLINEGON_H

#include "geometry.h"
#include "math_solver.h"
#include <vector>
#include <optional>

// Represents a piece of the boundary of the Visibility Diagram D.
struct RationalArc {
    // The generator vertex from P that defines the equation
    Point pivot_vertex; 

    // Angular domain (sector) in velocity space
    // alpha = atan2(v_r, v_q)
    double theta_start;
    double theta_end;

    // Helper for logic
    bool covers_angle(double theta) const {
        return theta >= theta_start && theta <= theta_end;
    }
};

// Implements the O(log n) Query Structure described in Theorem 1.
// Construction: O(n) (using LinearShortestPath)
// Query: O(log n) (using Binary Search on Monotone Sectors)
class SplinegonDiagram {
    const Polygon& P;
    const Trajectory& q_geom; // Base path q
    const Trajectory& r_geom; // Base path r

    // The ordered angular sectors partitioning the visibility plane.
    std::vector<RationalArc> lower_envelope_sectors;

    void construct_monotone_decomposition();

public:
    SplinegonDiagram(const Polygon& poly, const Trajectory& q, const Trajectory& r);

    // Queries the Splinegon boundary in O(log n) time.
    std::optional<double> shoot_ray(double v_q, double v_r) const;
};

#endif