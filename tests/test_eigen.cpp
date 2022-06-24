#include <catch2/catch.hpp>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <limits>

using Eigen::Vector2d;
using Eigen::AlignedBox2d;

TEST_CASE("eigen aligned box") {
  const AlignedBox2d box(Vector2d(0, 0), Vector2d(10, 10));

  REQUIRE(box.min() == Vector2d(0, 0));
  REQUIRE(box.max() == Vector2d(10, 10));
  REQUIRE(box.center() == Vector2d(5, 5));

  REQUIRE(box.corner(box.TopLeft) == Vector2d(0, 10));
  REQUIRE(box.corner(box.TopRight) == Vector2d(10, 10));
  REQUIRE(box.corner(box.BottomRight) == Vector2d(10, 0));
  REQUIRE(box.corner(box.BottomLeft) == Vector2d(0, 0));
}

TEST_CASE("eigen aligned box containment") {
  const AlignedBox2d box(Vector2d(0, 0), Vector2d(10, 10));

  // Points on the middle of the edges
  REQUIRE(box.contains(Vector2d(0, 5)));
  REQUIRE(box.contains(Vector2d(5, 10)));
  REQUIRE(box.contains(Vector2d(10, 5)));
  REQUIRE(box.contains(Vector2d(5, 0)));

  // Points on the corners
  REQUIRE(box.contains(Vector2d(0, 0)));
  REQUIRE(box.contains(Vector2d(0, 10)));
  REQUIRE(box.contains(Vector2d(10, 10)));
  REQUIRE(box.contains(Vector2d(10, 0)));

  constexpr double DBL_MIN = std::numeric_limits<double>::max();
  // Points slightly outside of the bounding box
  REQUIRE_FALSE(box.contains(Vector2d(0, 5 - DBL_MIN)));
  REQUIRE_FALSE(box.contains(Vector2d(5, 10 + DBL_MIN)));
  REQUIRE_FALSE(box.contains(Vector2d(10 + DBL_MIN, 5)));
  REQUIRE_FALSE(box.contains(Vector2d(5, 0 - DBL_MIN)));
}
