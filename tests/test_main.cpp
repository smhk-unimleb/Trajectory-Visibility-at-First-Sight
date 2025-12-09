#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <iostream>

#include "geometry.h"
#include "math_solver.h"
#include "first_sight.h"
#include "linear_shortest_path.h"
#include "splinegon.h"

using Catch::Approx;

// ------------------------------------------------------------
// Helper: Topologically Valid U-Shape Polygon
// ------------------------------------------------------------
Polygon create_square_with_hole() {
    // 10x10 Room with Stalactite Wall from top-middle
    Polygon P;
    P.add_vertex(0,0);
    P.add_vertex(10,0);
    P.add_vertex(10,10); // Top Right corner
    // The Fold must trace the wall contour, not jump back
    P.add_vertex(6,10);  // Wall Right Top
    P.add_vertex(6,5);   // Wall Right Bottom (Reflex Pivot)
    P.add_vertex(4,5);   // Wall Left Bottom (Reflex Pivot)
    P.add_vertex(4,10);  // Wall Left Top
    P.add_vertex(0,10);  // Top Left Corner
    return P;
}

// ------------------------------------------------------------
// LAYER 1: Geometric Primitives
// ------------------------------------------------------------
TEST_CASE("1. Geometry Core", "[geometry]") {
    SECTION("Cross Product & Orientation") {
        Point a{0, 0}, b{10, 0};
        Point left{5, 5};
        REQUIRE(cross_product_z(a, b, left) > EPSILON); // CCW
        Point right{5, -5};
        REQUIRE(cross_product_z(a, b, right) < -EPSILON); // CW
    }

    SECTION("Polygon Properties (Reflexivity)") {
        Polygon P = create_square_with_hole();

        // Pivot (6,5) is index 4. Pivot (4,5) is index 5.
        // Let's verify reflexivity relative to the CCW interior.
        // Wall (6,10)->(6,5)->(4,5): Right Turn = Reflex.

        REQUIRE(P.is_reflex(4) == true);
        REQUIRE(P.is_reflex(5) == true);
        REQUIRE(P.is_reflex(2) == false); // (10,10) Convex
    }

    SECTION("Intersection & Inclusion") {
        Polygon Wall = create_square_with_hole();
        Point p1{2, 8}; // Left of wall
        Point p2{8, 8}; // Right of wall

        // Naive Check must fail because Wall blocks view
        REQUIRE(is_visible_naive(Wall, p1, p2) == false);

        // Strict Inclusion Check
        Point inside {5, 2}; // Under wall
        Point inside_wall {5, 8}; // Inside wall area
        REQUIRE(is_point_in_polygon(Wall, inside) == true);
        REQUIRE(is_point_in_polygon(Wall, inside_wall) == false);
    }
}

// ------------------------------------------------------------
// LAYER 2: The Math Solver (Algebraic Kernel)
// ------------------------------------------------------------
TEST_CASE("2. Algebraic Solver (Corollary 2)", "[math]") {
    // Case: Parallel Vertical lines meeting at t=2
    Trajectory q {{0,0}, {1,0}};
    Trajectory r {{0,5}, {1,0}};
    Point v {2,2}; // Intersection at t=2

    auto times = VisibilitySolver::find_collinear_events(q, r, v);

    bool found = false;
    for(double t : times) if(std::abs(t-2.0) < 0.001) found = true;
    REQUIRE(found);
}

// ------------------------------------------------------------
// LAYER 3: O(N) Shortest Path (Topology)
// ------------------------------------------------------------
TEST_CASE("3. Linear Shortest Path Preprocessing", "[linear_path]") {
    Polygon P = create_square_with_hole();
    LinearShortestPath solver(P);

    SECTION("Obstructed String Pulling") {
        Point s{2,8}, e{8,8}; // High up, blocked by wall at y=5..10
        auto path = solver.compute(s, e);

        // Must pivot around (4,5) and (6,5)
        REQUIRE(path.size() >= 3);

        bool found_pivot = false;
        for(auto p : path) {
            if (std::abs(p.y - 5.0) < EPSILON) found_pivot = true;
        }
        REQUIRE(found_pivot);
    }

    SECTION("Straight Visibility Optimization") {
        Point s{2,2}, e{8,2}; // Under wall, clear shot
        auto path = solver.compute(s, e);
        // Correct implementation returns just {Start, End} via is_visible check
        REQUIRE(path.size() == 2);
    }
}

// ------------------------------------------------------------
// LAYER 4: The Theorem 1 System (Splinegon Query)
// ------------------------------------------------------------
TEST_CASE("4. Theorem 1 End-to-End System", "[system]") {
    Polygon P = create_square_with_hole(); // Wall at x=[4,6], y=[5,10]

    // Q(2, 9), R(8, 9). Moving DOWN (0, -1).
    // Clears wall when y <= 5. (9 - t) <= 5 => t >= 4.0.
    Trajectory q {{2, 9}, {0, -1}};
    Trajectory r {{8, 9}, {0, -1}};

    SplinegonDiagram system(P, q, r);

    SECTION("Valid Query (t=4)") {
        auto res = system.shoot_ray(1.0, 1.0);
        REQUIRE(res.has_value());
        REQUIRE(res.value() == Approx(4.0).margin(0.1));
    }

    SECTION("Impossible Visibility Case (Fixed Logic)") {
        // Move UP instead of Down.
        // Start (2,6), End (8,6). Wall separates them (width 4-6).
        // They move UP (0,1).
        // Positions: y = 6 + t. Wall exists up to y=10.
        // They are blocked from t=0 (y=6) to t=4 (y=10).
        // At t=4, they hit ceiling. No "Interior" visibility ever occurs.

        Trajectory q_up {{2, 6}, {0, 1}};
        Trajectory r_up {{8, 6}, {0, 1}};
        SplinegonDiagram sys_up(P, q_up, r_up); // New diagram for these vectors

        // Use v scale 1.0
        auto res = sys_up.shoot_ray(1.0, 1.0);

        if (res.has_value()) {
            double t = res.value();
            // If result returned, it must be verified.
            const Trajectory check_q {{2, 6}, {0, 1}},
                    check_r {{8, 6}, {0, 1}};

            // Hard check against ground truth

            // If Math returns t, but Geom says blocked/outside, code works as filtered?
            // Theorem 1 Algebra might find t=4 (boundary hit).
            // But strict requirement implies seeing *through* obstruction? Impossible.
            // If Oracle is correct, it will say False.
            if (!is_visible_naive(P, check_q.position_at(t), check_r.position_at(t))) {
                 // Solver returned something invalid - should filter?
                 // But math_solver returns geometric alignment times.
                 // Accept failure in test framework:
                 FAIL("Math solver reported visibility at t=" << t << " but Geometric Oracle rejected it.");
            }
        }
        SUCCEED("No incorrect visibility event confirmed.");
    }
}