#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "geometry.h"
#include "math_solver.h"
#include "first_sight.h"

using Catch::Approx;

TEST_CASE("Polygon Geometry Logic", "[geometry]") {
    Polygon P;
    // Correct setup: Outer box 10x10 with "Stalactite" Wall in top middle
    P.add_vertex(0,10); P.add_vertex(0,0); P.add_vertex(10,0); P.add_vertex(10,10);
    // Wall cutout: x goes in to 6, down to 4, over to 4, up to 10
    P.add_vertex(6,10); P.add_vertex(6,4); P.add_vertex(4,4); P.add_vertex(4,10);

    SECTION("Winding and Reflexivity") {
        // Vertex 5 is (4,4). Inner Corner. Should be reflex.
        REQUIRE(P.is_reflex(6) == true);
    }

    SECTION("Point Inclusion Check") {
        REQUIRE(is_point_in_polygon(P, {2, 8}) == true); // Left Zone
        REQUIRE(is_point_in_polygon(P, {8, 8}) == true); // Right Zone
        REQUIRE(is_point_in_polygon(P, {5, 6}) == false); // Inside Wall
        REQUIRE(is_point_in_polygon(P, {5, 2}) == true); // Below Wall
    }
}

TEST_CASE("Trajectory Collinearity Solver", "[math]") {
    // Check known event generation at t=5
    Trajectory q {{0,0}, {1,1}};
    Trajectory r {{0,10}, {1,0}};
    Point v {5,5};

    auto times = VisibilitySolver::find_collinear_events(q, r, v);

    bool found_5 = false;
    for(auto t : times) if(std::abs(t - 5.0) < 1e-4) found_5 = true;
    REQUIRE(found_5);
}

TEST_CASE("End-to-End System: First Visibility", "[system]") {
    Polygon P;
    P.add_vertex(0,0); P.add_vertex(10,0); P.add_vertex(10,10);
    P.add_vertex(6,10); P.add_vertex(6,4); // Wall R
    P.add_vertex(4,4); P.add_vertex(4,10); // Wall L
    P.add_vertex(0,10);

    // Q and R start high up (y=8), descending.
    // They are separated by wall (y in [4,10]).
    // Line of sight clears exactly at y=4.
    // y(t) = 8 - t. 4 = 8 - t => t = 4.0.

    Trajectory q {{2, 8}, {0, -1}};
    Trajectory r {{8, 8}, {0, -1}};

    FirstSightFinder finder(P);
    auto result = finder.find_first_sight(q, r);

    REQUIRE(result.has_value());
    REQUIRE(result.value() == Approx(4.0).margin(0.1));
}