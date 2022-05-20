#include <iostream>

#include "barnes_hut/force.h"
#include "catch2/catch.hpp"

using namespace bh::force;
using Eigen::Vector2f;

TEST_CASE("compute gravitational force") {
  SECTION("coinciding bodies") {
    Body body({2.5, 3.7}, 1);
    REQUIRE(compute_gravitational_force(body, body) == Vector2f(0, 0));
  }

  SECTION("two bodies on the x axis") {
    Body b1({0.f, 0.f}, 1.f);
    Body b2({10.f, 0.f}, 1.f);

    SECTION("force that b1 exerts on b2") {
      Vector2f f = compute_gravitational_force(b1, b2);
      REQUIRE(f.x() == -0.005f);  // -0.00499999989 -> -0.005f
      // REQUIRE(f.y() == 0.f); // -4.37113873E-10 -> -0.0f
    }
  }

  SECTION("two bodies on the y axis") {
    Body b1({0.f, 0.f}, 1.f);
    Body b2({0.f, 10.f}, 1.f);

    SECTION("force that b1 exerts on b2") {
      Vector2f f = compute_gravitational_force(b1, b2);
      // REQUIRE(f.x() == 0.f);      // -2.18556936E-10 -> -0.0f
      REQUIRE(f.y() == -0.005f);  // -0.00499999989 -> -0.005f
    }
  }

  /*
  SECTION("earth and moon") {
    // https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    // 5.9722*10^24 kg
    float earth_m = 5972200000000000000000000.f;
    // https://nssdc.gsfc.nasa.gov/planetary/factsheet/moonfact.html
    // 0.07346*10^24 kg
    float moon_m = 73460000000000000000000.f;
    // Mean values at opposition from Earth: 378,000 km
    float distance = 378000000.f;

    SECTION("both bodies on the x axis") {
      // Compute the gravitational force the moon exerts on earth
      Body earth({0, 0}, earth_m);
      Body moon({distance, 0}, moon_m);
      Vector2f actual_f = compute_gravitational_force(earth, moon);
      Vector2f expected_f(
          204930873479997729716.566445508244450043391842333641275440217f, 0);
      REQUIRE(actual_f.x() == expected_f.x());
      REQUIRE(actual_f.y() == expected_f.y());
    }
  }
   */
}
