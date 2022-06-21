#include <catch2/catch.hpp>
#include <iostream>

#include "force.h"

using bh::Body;
using bh::Node;
using Eigen::Vector2d;

TEST_CASE("compute gravitational force") {
  SECTION("coinciding bodies") {
    Body body({2.5, 3.7}, 1);
    REQUIRE(bh::compute_gravitational_force(body, body) == Vector2d(0, 0));
  }

  SECTION("two bodies on the x axis") {
    Body b1({0, 0}, 1);
    Body b2({10, 0}, 1);

    SECTION("force that b1 exerts on b2") {
      Vector2d f = bh::compute_gravitational_force(b1, b2, 0.5);
      REQUIRE(f.x() == -0.005);  // -0.00499999989 -> -0.005f
      // REQUIRE(f.y() == 0); // -4.37113873E-10 -> -0.0f
    }
  }

  SECTION("two bodies on the y axis") {
    Body b1({0, 0}, 1);
    Body b2({0, 10}, 1);

    SECTION("force that b1 exerts on b2") {
      Vector2d f = bh::compute_gravitational_force(b1, b2, 0.5);
      // REQUIRE(f.x() == 0);      // -2.18556936E-10 -> -0.0f
      REQUIRE(f.y() == -0.005);  // -0.00499999989 -> -0.005f
    }
  }

  SECTION("two bodies, 45°") {
    Body b1({0, 0}, 1);
    Body b2({10, 10}, 1);

    SECTION("force that b1 exerts on b2") {
      Vector2d f = bh::compute_gravitational_force(b1, b2, 0.5);
      REQUIRE(f.x() == Approx(-0.00176776695297));  // using Desmos calculator
      REQUIRE(f.y() == Approx(-0.00176776695297));
    }
  }

  SECTION("three bodies") {
    Body left({-10, 0}, 1);
    Body center({0, 0}, 1);
    Body right({10, 0}, 1);

    Vector2d left_f = bh::compute_gravitational_force(left, center, 0.5);
    Vector2d right_f = bh::compute_gravitational_force(right, center, 0.5);
    Vector2d sum_f = left_f + right_f;
    REQUIRE(sum_f.x() == 0);
    // REQUIRE(sum_f.y() == 0); // -0.0f == 0.0f
  }

  /*
  SECTION("earth and moon") {
    // https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    // 5.9722*10^24 kg
    double earth_m = 5972200000000000000000000;
    // https://nssdc.gsfc.nasa.gov/planetary/factsheet/moonfact.html
    // 0.07346*10^24 kg
    double moon_m = 73460000000000000000000;
    // Mean values at opposition from Earth: 378,000 km
    double distance = 378000000;

    SECTION("both bodies on the x axis") {
      // Compute the gravitational force the moon exerts on earth
      Body earth({0, 0}, earth_m);
      Body moon({distance, 0}, moon_m);
      Vector2d actual_f = compute_gravitational_force(earth, moon);
      Vector2d expected_f(
          204930873479997729716.566445508244450043391842333641275440217f, 0);
      REQUIRE(actual_f.x() == expected_f.x());
      REQUIRE(actual_f.y() == expected_f.y());
    }
  }
   */
}

TEST_CASE("compute exact net force on body") {
  std::vector<Body> bodies;
  Vector2d f;

  // Quadtree is empty
  f = compute_exact_net_force_on_body(bodies, Body{{0, 0}, 1});
  REQUIRE(f == Vector2d(0, 0));

  bodies.emplace_back(Vector2d{5, 0}, 1);
  f = compute_exact_net_force_on_body(bodies, {{0, 0}, 1}, 0.5);
  REQUIRE(f.x() == 0.02);  // -0.00499999989 -> -0.005f
  REQUIRE(f.y() == 0);     // -4.37113873E-10 -> -0.0f

  bodies.emplace_back(Vector2d{-5, 0}, 2);
  f = compute_exact_net_force_on_body(bodies, {{0, 0}, 1}, 0.5);
  REQUIRE(f.x() == -0.02);
  // REQUIRE(f.y() == 0); // -0.0f == 0.0f
}

TEST_CASE("compute approximate net force on body") {
  Node root(Vector2d(-10, -10), Vector2d(10, 10));

  root.insert({{7.5, -2.5}, 0.5});
  root.insert({{2.5, -7.5}, 0.5});

  Vector2d f = compute_approximate_net_force_on_body(root, {{-10, 10}, 1}, 0.5);
  REQUIRE(f.x() == Approx(0.000785674201318));
  REQUIRE(f.y() == Approx(-0.000785674201318));
}
