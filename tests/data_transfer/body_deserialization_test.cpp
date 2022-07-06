#include "body_deserialization.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Deserialize a single body") {
  const auto body = bh::mpi::Body{0.5, 1.5, -2, -3, 0.25};
  const auto deserialized = bh::deserialize_body(body);

  REQUIRE(deserialized.m_position == Eigen::Vector2d{0.5, 1.5});
  REQUIRE(deserialized.m_mass == 0.25);
  REQUIRE(deserialized.m_velocity == Eigen::Vector2d{-2, -3});
}

TEST_CASE("Deserialize a vector of bodies") {
  std::vector<bh::mpi::Body> bodies{{0.5, 1.5, -2, -3, 0.25},
                                    {1.5, 2.5, -3, -4, 1.25}};
  auto deserialized = bh::deserialize_bodies(bodies);

  REQUIRE(deserialized.size() == 2);

  const auto &first = deserialized[0];
  const auto &second = deserialized[1];

  REQUIRE(first.m_position == Eigen::Vector2d{0.5, 1.5});
  REQUIRE(first.m_mass == 0.25);
  REQUIRE(first.m_velocity == Eigen::Vector2d{-2, -3});

  REQUIRE(second.m_position == Eigen::Vector2d{1.5, 2.5});
  REQUIRE(second.m_mass == 1.25);
  REQUIRE(second.m_velocity == Eigen::Vector2d{-3, -4});
}
