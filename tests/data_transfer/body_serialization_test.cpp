#include "body_serialization.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Serialize a single body") {
  const auto body = bh::Body{{0.5, 1.5}, 0.25, {-2, -3}};
  const auto serialized = bh::serialize_body(body);

  REQUIRE(serialized.position_x == 0.5);
  REQUIRE(serialized.position_y == 1.5);
  REQUIRE(serialized.mass == 0.25);
  REQUIRE(serialized.velocity_x == -2);
  REQUIRE(serialized.velocity_y == -3);
}

TEST_CASE("Serialize a vector of bodies") {
  std::vector<bh::Body> bodies{{{0.5, 1.5}, 0.25, {-2, -3}},
                               {{1.5, 2.5}, 1.25, {-3, -4}}};
  auto serialized = bh::serialize_bodies(bodies);

  REQUIRE(serialized.size() == 2);

  const auto &first = serialized[0];
  const auto &second = serialized[1];

  REQUIRE(first.position_x == 0.5);
  REQUIRE(first.position_y == 1.5);
  REQUIRE(first.mass == 0.25);
  REQUIRE(first.velocity_x == -2);
  REQUIRE(first.velocity_y == -3);

  REQUIRE(second.position_x == 1.5);
  REQUIRE(second.position_y == 2.5);
  REQUIRE(second.mass == 1.25);
  REQUIRE(second.velocity_x == -3);
  REQUIRE(second.velocity_y == -4);
}
