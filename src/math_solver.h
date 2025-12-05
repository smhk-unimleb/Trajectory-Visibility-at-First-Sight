#ifndef TV_MATH_SOLVER_H
#define TV_MATH_SOLVER_H

#include "geometry.h"
#include <vector>

struct Trajectory {
    Point start;
    Vector2D v;

    Point position_at(double t) const {
        return start + (v * t);
    }
};

class VisibilitySolver {
public:
    // Solves At^2 + Bt + C = 0 for real non-negative t.
    // Handles degenerate linear cases where A=0.
    static std::vector<double> solve_quadratic_time(const double& A, const double& B, const double& C);

    // Identifies critical timestamps when q(t), r(t), and v become collinear.
    // See paper Section 2, Corollary 2.
    static std::vector<double> find_collinear_events(
        const Trajectory& q, 
        const Trajectory& r, 
        const Point& reflex_vertex
    );
};

#endif // TV_MATH_SOLVER_H