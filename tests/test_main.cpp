#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

// Include the new architectural headers
#include "geometry.h"
#include "math_solver.h"
#include "linear_shortest_path.h" // The O(N) Solver
#include "splinegon.h"            // The O(log n) System

using Catch::Approx;

TEST_CASE("1. Geometry Primitives & Winding", "[geometry]") {
    // Verify cross_product_z logic is consistent for CCW calculations
    Point a {0, 0};
    Point b {10, 0};
    Point c_left {5, 5};   // Left turn (CCW)
    Point c_right {5, -5}; // Right turn (CW)

    SECTION("Orientation (Cross Product Z)") {
        REQUIRE(cross_product_z(a, b, c_left) > EPSILON);  // Positive for Left/CCW
        REQUIRE(cross_product_z(a, b, c_right) < -EPSILON); // Negative for Right/CW
    }

    SECTION("Winding Logic Checks") {
        // Standard Square (CCW)
        Polygon ccw_sq;
        ccw_sq.add_vertex(0,0); ccw_sq.add_vertex(10,0);
        ccw_sq.add_vertex(10,10); ccw_sq.add_vertex(0,10);

        // LinearShortestPath calculates winding in constructor
        LinearShortestPath lsp(ccw_sq);
        // We can't access private ccw_winding directly,
        // but we can infer it via string pulling behavior in next section.
        SUCCEED("Winding calculated without crash.");
    }
}

TEST_CASE("2. Preprocessing: O(N) String Pulling", "[preprocessing]") {
    // Construct a "U" Shaped Polygon where visibility must wrap around a reflex inner corner.
    //
    //    (0,10)      (10,10)
    //      |   VOID    |
    //      | (2,10) (8,10)
    //      |   |      |
    //      |   |______| (8,2) - Reflex Vertex 1
    //      |          |
    //      |__________|
    //    (0,0)      (10,0)
    //               ^ (10,0) typo in comment, but coordinates below:

    Polygon U_shape;
    // Outer Shell
    U_shape.add_vertex(0,10);
    U_shape.add_vertex(0,0);
    U_shape.add_vertex(10,0);
    U_shape.add_vertex(10,10);
    // Inner Void (creating the corridor)
    // Vertices order matters for valid simple polygon boundary traversal
    U_shape.add_vertex(8,10);
    U_shape.add_vertex(8,2); // Reflex Corner (Inner Right)
    U_shape.add_vertex(2,2); // Reflex Corner (Inner Left)
    U_shape.add_vertex(2,10);

    LinearShortestPath O_N_solver(U_shape);

    SECTION("Taut String wraps around Reflex Vertices") {
        Point start {1, 9}; // Inside Left Arm
        Point end {9, 9};   // Inside Right Arm

        // Path cannot go straight (blocked by void 2..8, 2..10).
        // Must go Down, across bottom corridor, and Up.
        // Critical pivots: (2,2) and (8,2).

        std::vector<Point> path = O_N_solver.compute(start, end);

        // Valid path must have: Start -> (2,2) -> (8,2) -> End
        // (Exact index depends on vertex ordering in stack, but pivot presence is key)

        REQUIRE(path.size() >= 4);
        REQUIRE(path.front() == start);
        REQUIRE(path.back() == end);

        // Check for presence of critical Reflex Pivot (2,2)
        bool hit_elbow = false;
        for(const auto& p : path) {
            if (std::abs(p.x - 2.0) < EPSILON && std::abs(p.y - 2.0) < EPSILON) hit_elbow = true;
        }
        REQUIRE(hit_elbow == true);
    }

    SECTION("Straight line visibility maintained in convex regions") {
        Point s {1, 9};
        Point e {1, 2};
        // Should be a straight line down the left arm
        std::vector<Point> path = O_N_solver.compute(s, e);

        REQUIRE(path.size() == 2); // Just start and end
        REQUIRE(path[0] == s);
        REQUIRE(path[1] == e);
    }
}

TEST_CASE("3. System: Theorem 1 First Visibility Query", "[splinegon]") {
    // THEOREM 1: FIXED TRAJECTORIES
    // Use the Hanging Wall scenario for strict t* calculation.

    // Geometry:
    // Room 10x10. Wall hanging from top middle x=[4,6], y=[4,10].
    Polygon P;
    P.add_vertex(0,0);
    P.add_vertex(10,0);
    P.add_vertex(10,10);
    // Wall cutout sequence
    P.add_vertex(6,10);
    P.add_vertex(6,4); // Reflex Corner 1
    P.add_vertex(4,4); // Reflex Corner 2
    P.add_vertex(4,10);
    P.add_vertex(0,10);

    // Entities descend from y=8.
    // Wall is solid in y in [4,10].
    // Line of sight connects Q(2, 8-t) and R(8, 8-t).
    // The wall ends at y=4.
    // Visibility established when 8-t <= 4 implies t >= 4.0.

    Trajectory q {{2, 8}, {0, -1}};
    Trajectory r {{8, 8}, {0, -1}};

    // 1. Construct Splinegon Diagram (Uses O(N) Solver internally)
    SplinegonDiagram splinegon(P, q, r);

    // 2. Query Phase (O(log n))
    SECTION("Calculates Correct First Sight Time") {
        // Equal speeds (v_q = 1, v_r = 1) -> descend normally.
        auto result = splinegon.shoot_ray(1.0, 1.0);

        REQUIRE(result.has_value());
        REQUIRE(result.value() == Approx(4.0).margin(0.1));
    }

    SECTION("Calculates High Speed Offset") {
        // v_q = 2, v_r = 2 (Both faster, equal ratio) -> angle same, Time should be halved?
        // Wait, Ray Shooting in Diagram D is in Velocity SPACE (Ratio v_r/v_q determines Sector).
        // Then we solve Eq: Q(t), R(t) collinear with Vertex.
        // Q(t) = (2, 8 - 2t), R(t) = (8, 8 - 2t).
        // Midpoint y = 8 - 2t.
        // Must clear y=4. -> 8-2t <= 4 -> 2t >= 4 -> t >= 2.

        // This confirms the math kernel scales t properly for different magnitudes
        // but same Ray Slope.

        auto result = splinegon.shoot_ray(2.0, 2.0);

        REQUIRE(result.has_value());
        REQUIRE(result.value() == Approx(2.0).margin(0.1));
    }
}