#include "first_sight.h"
#include <algorithm>

bool FirstSightFinder::verify_visibility_at(const double t, const Trajectory& q_traj, const Trajectory& r_traj) const {
    Point q_pos = q_traj.position_at(t);
    Point r_pos = r_traj.position_at(t);

    // 1. Boundary Intersection Check
    if (!is_visible_naive(P, q_pos, r_pos))
        return false;

    // 2. Interior Check (Midpoint must be inside P to rule out external tangencies)
    Point mid = (q_pos + r_pos) / 2.0;
    if (!is_point_in_polygon(P, mid))
        return false;

    return true;
}

std::optional<double> FirstSightFinder::find_first_sight(const Trajectory& q, const Trajectory& r) const {
    double min_time = std::numeric_limits<double>::infinity();
    bool found_valid = false;

    // Initial configuration check
    if (verify_visibility_at(0.0, q, r)) {
        return 0.0;
    }

    // Process all potential pivot vertices
    for (size_t i = 0; i < P.size(); ++i) {
        Point v = P.get_vertex(i);
        
        // Solve the algebraic condition for collinearity

        for (const double t : VisibilitySolver::find_collinear_events(q, r, v)) {
            // Discard past events or events later than current best
            if (t < 0 || t >= min_time)
                continue;

            if (verify_visibility_at(t, q, r)) {
                min_time = t;
                found_valid = true;
            }
        }
    }

    if (found_valid)
        return min_time;
    return std::nullopt;
}
