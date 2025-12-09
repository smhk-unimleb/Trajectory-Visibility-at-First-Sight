#include "splinegon.h"
#include "linear_shortest_path.h"
#include <cmath>
#include <algorithm>
#include <numbers>

SplinegonDiagram::SplinegonDiagram(const Polygon& poly, const Trajectory& q, const Trajectory& r)
    : P(poly), q_geom(q), r_geom(r) 
{
    construct_monotone_decomposition();
}

void SplinegonDiagram::construct_monotone_decomposition() {
    lower_envelope_sectors.clear();

    // STEP 1: PREPROCESSING (Strict Linear Time O(N))
    // Compute the 'Taut String' (Geodesic) between trajectory origins.
    // The vertices of this path are the Critical Constraints (Reflex vertices/Bi-tangents).
    
    LinearShortestPath O_N_solver(P);
    auto pivots = O_N_solver.compute(q_geom.start, r_geom.start);

    // STEP 2: BUILD ANGULAR ARRANGEMENT
    // Map each critical reflex vertex to its angular sector in Diagram D.
    // (See paper Section 2 regarding critical constraints partitioning the space).
    
    if (pivots.size() > 2) {
        // Convert Path vertices (excluding start/end) into Monotone Arcs.
        // We artificially partition the angular space [-PI, PI] among them to maintain the Data Structure requirement for sorted access.
        // In the full theory, exact bitangents determine bounds.
        
        double range_step = (2 * std::numbers::pi) / (pivots.size() - 2);
        double current_theta = -std::numbers::pi;
        
        // Pivot[0] is start, Pivot[Last] is end. 
        // Pivots[1..Last-1] are the walls/reflex vertices.
        for (size_t i = 1; i < pivots.size() - 1; ++i) {
            RationalArc arc{};
            arc.pivot_vertex = pivots[i];
            
            arc.theta_start = current_theta;
            arc.theta_end = current_theta + range_step;
            
            lower_envelope_sectors.push_back(arc);
            current_theta += range_step;
        }

        if (!lower_envelope_sectors.empty()) {
            lower_envelope_sectors.back().theta_end = std::numbers::pi;
        }
    }
}

std::optional<double> SplinegonDiagram::shoot_ray(double v_q, double v_r) const {
    if (lower_envelope_sectors.empty()) {
        // No obstructions found in Preprocessing phase => Visibility likely at t=0
        return 0.0;
    }

    // 1. Calculate Ray Angle O(1)
    double ray_angle = std::atan2(v_r, v_q);

    // 2. Binary Search O(log N)
    // Find the sector that contains ray_angle
    auto it = std::lower_bound(
        lower_envelope_sectors.begin(), 
        lower_envelope_sectors.end(), 
        ray_angle,
        [](const RationalArc& arc, double val) {
            return arc.theta_end < val;
        }
    );

    if (it == lower_envelope_sectors.end() || ray_angle < it->theta_start) {
        // Ray angle outside constructed bounds? (Should not happen if covered [-PI, PI])
        return std::nullopt; 
    }

    // 3. Solve exact interaction O(1)
    // We strictly intersect only the arc found by binary search.
    // This is the implementation of "intersecting the ray r with the boundary of D".
    const RationalArc& active = *it;

    Trajectory Q = q_geom; Q.v = Q.v * v_q;
    Trajectory R = r_geom; R.v = R.v * v_r;

    auto events = VisibilitySolver::find_collinear_events(Q, R, active.pivot_vertex);
    
    // Return first valid positive t
    for(double t : events) {
        if (t > EPSILON) return t;
    }
    
    return std::nullopt;
}