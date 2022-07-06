#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "force.h"

SCENARIO("Compute gravitational force, uncommon cases") {
  GIVEN("A single body") {
    const auto body = bh::Body{{0.5, 2.5}, 0.25};

    THEN("It does not exert any gravitational force on itself") {
      const auto force = bh::compute_gravitational_force(body, body);
      REQUIRE(force == Eigen::Vector2d{0, 0});
    }
  }

  GIVEN("Two bodies that coincide") {
    // they also have different masses
    const auto first = bh::Body{{1.5, 2.5}, 0.2};
    const auto second = bh::Body{{1.5, 2.5}, 1.4};

    THEN("They do not exert any gravitational force on each other") {
      const auto force_on_first = bh::compute_gravitational_force(second, first);
      const auto force_on_second = bh::compute_gravitational_force(first, second);

      REQUIRE(force_on_first == Eigen::Vector2d{0, 0});
      REQUIRE(force_on_second == Eigen::Vector2d{0, 0});
    }
  }
}

SCENARIO("Compute gravitational force, common cases") {
  GIVEN("Two bodies on the horizontal x axis") {
    const auto body_on_the_left = bh::Body{{-1, 0}, 0.5};
    const auto body_on_the_right = bh::Body{{1, 0}, 2.5};

    THEN("They exert on each other a gravitational force") {
      // magnitude: (G * m1 * m2) / d² = (1 * 0.5 * 2.5) / 2² = 0.3125

      const auto force_on_the_body_on_the_left = bh::compute_gravitational_force(body_on_the_right, body_on_the_left, 1);
      const auto force_on_the_body_on_the_right = bh::compute_gravitational_force(body_on_the_left, body_on_the_right, 1);

      REQUIRE(force_on_the_body_on_the_left.x() == 0.3125);
      REQUIRE(force_on_the_body_on_the_right.x() == -0.3125);
    }
  }

  GIVEN("Two bodies on the vertical y axis") {
    const auto body_below = bh::Body{{0, -1}, 0.5};
    const auto body_above = bh::Body{{0, 1}, 2.5};

    THEN("They exert on each other a gravitational force") {
      // magnitude: (G * m1 * m2) / d² = (1 * 0.5 * 2.5) / 2² = 0.3125

      const auto force_on_the_body_below = bh::compute_gravitational_force(body_above, body_below, 1);
      const auto force_on_the_body_above = bh::compute_gravitational_force(body_below, body_above, 1);

      REQUIRE(force_on_the_body_below.y() == 0.3125);
      REQUIRE(force_on_the_body_above.y() == -0.3125);
    }
  }

  GIVEN("Two bodies on the y=x axis (angle: 45°)") {
    const auto body_below = bh::Body{{-1, -1}, 0.5};
    const auto body_above = bh::Body{{1, 1}, 2.5};

    THEN("They exert on each other a gravitational force") {
      // magnitude: (G * m1 * m2) / d² = (1 * 0.5 * 2.5) / (√2)² = 0.15625
      // cos(45°) * magnitude = 0.11048543456
      // sin(45°) * magnitude = 0.11048543456

      const auto force_on_the_body_below = bh::compute_gravitational_force(body_above, body_below, 1);
      const auto force_on_the_body_above = bh::compute_gravitational_force(body_below, body_above, 1);

      REQUIRE(force_on_the_body_below.x() == Catch::Approx(0.11048543456));
      REQUIRE(force_on_the_body_below.y() == Catch::Approx(0.11048543456));

      REQUIRE(force_on_the_body_above.x() == Catch::Approx(-0.11048543456));
      REQUIRE(force_on_the_body_above.y() == Catch::Approx(-0.11048543456));
    }
  }

  GIVEN("Two bodies on a line with angle 30°") {
    constexpr auto PI_6 = M_PI / 6;
    const auto body_below = bh::Body{{-1, -std::tan(PI_6)}, 0.5};
    const auto body_above = bh::Body{{1, std::tan(PI_6)}, 2.5};

    THEN("They exert on each other a gravitational force") {
      // magnitude: (G * m1 * m2) / d² = (1 * 0.5 * 2.5) / (√(2²+1²))² = 2.2360679775
      // cos(30°) * magnitude = 0.202974704012
      // sin(30°) * magnitude = 0.1171875

      const auto force_on_the_body_below = bh::compute_gravitational_force(body_above, body_below, 1);
      const auto force_on_the_body_above = bh::compute_gravitational_force(body_below, body_above, 1);

      // REQUIRE(force_on_the_body_below.x() == Catch::Approx(0.202974704012));
      // REQUIRE(force_on_the_body_below.y() == Catch::Approx(0.1171875));

      // REQUIRE(force_on_the_body_above.x() == Catch::Approx(0.202974704012));
      // REQUIRE(force_on_the_body_above.y() == Catch::Approx(0.1171875));
    }
  }
}
