#include "math_solver.h"
#include <algorithm>
#include <cmath>

std::vector<double> VisibilitySolver::solve_quadratic_time(const double& A, const double& B, const double& C) {
    std::vector<double> solutions;

    // Linear case A ~ 0 (e.g. parallel speed vectors)
    if (std::abs(A) < EPSILON) {
        if (std::abs(B) > EPSILON) {
            double t = -C / B;
            if (t > -EPSILON) solutions.push_back(std::max(0.0, t));
        }
        return solutions;
    }

    double discriminant = B * B - 4 * A * C;
    if (discriminant < -EPSILON) return solutions;

    const double sqrt_d = std::sqrt(std::max(0.0, discriminant));
    const double t1 = (-B - sqrt_d) / (2 * A);
    const double t2 = (-B + sqrt_d) / (2 * A);

    if (t1 > -EPSILON) solutions.push_back(std::max(0.0, t1));
    if (t2 > -EPSILON) solutions.push_back(std::max(0.0, t2));

    std::ranges::sort(solutions);
    const auto last = std::ranges::unique(solutions, [](double a, double b){ return std::abs(a-b) < EPSILON; }).begin();
    solutions.erase(last, solutions.end());

    return solutions;
}

std::vector<double> VisibilitySolver::find_collinear_events(
    const Trajectory& t_q, const Trajectory& t_r, const Point& v)
{
    const double xq0 = t_q.start.x; const double yq0 = t_q.start.y;
    const double vqx = t_q.v.x;     const double vqy = t_q.v.y;
    const double xr0 = t_r.start.x; const double yr0 = t_r.start.y;
    const double vrx = t_r.v.x;     const double vry = t_r.v.y;
    const double xv = v.x;          const double yv = v.y;

    const double dx_q = xq0 - xv; const double dy_q = yq0 - yv;
    const double dx_r = xr0 - xv; const double dy_r = yr0 - yv;

    // A = v_q x v_r (z-component)
    double A = vqx * vry - vqy * vrx;

    // B derived from cross product expansion
    double B = (dx_q * vry + vqx * dy_r) - (dy_q * vrx + vqy * dx_r);

    // C constant spatial terms
    double C = dx_q * dy_r - dy_q * dx_r;

    return solve_quadratic_time(A, B, C);
}