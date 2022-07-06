#include <catch2/catch_test_macros.hpp>

#include "loader.h"

TEST_CASE("Read a file with two bodies") {
  // body-1.txt's content is as follows:
  // 2
  // 2.5 3.2 0.25 4.1 5.2
  // 7.3 8 1.4 6.5 3
  const auto bodies = bh::load_bodies(BODY_TWO);
  REQUIRE(bodies.size() == 2);

  const auto& first = bodies[0];
  const auto& second = bodies[1];

  REQUIRE(first.m_position == Eigen::Vector2d{2.5, 3.2});
  REQUIRE(first.m_mass == 0.25);
  REQUIRE(first.m_velocity == Eigen::Vector2d{4.1, 5.2});

  REQUIRE(second.m_position == Eigen::Vector2d{7.3, 8});
  REQUIRE(second.m_mass == 1.4);
  REQUIRE(second.m_velocity == Eigen::Vector2d{6.5, 3});
}

TEST_CASE("Read a body with negative mass throws exception") {
  // body-bad.txt's content is as follows:
  // 1
  // 2.5 3.2 -0.25 4.1 5.2
  REQUIRE_THROWS(bh::load_bodies(BODY_BAD));
}
