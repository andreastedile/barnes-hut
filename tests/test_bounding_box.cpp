#include <vector>

#include "body.h"
#include "bounding_box.h"
#include "catch2/catch.hpp"

TEST_CASE("compute minimum bounding box") {
  std::vector<bh::Body> nodes;
  nodes.push_back({{7, 4}, 0});      // top-right node
  nodes.push_back({{2, 3}, 0});      // bottom-left node
  nodes.push_back({{4.5, 3.5}, 0});  // somewhat in the center
  auto box = bh::compute_minimum_bounding_box(nodes);
  REQUIRE(box.min() == Vector2d(2, 3));
  REQUIRE(box.max() == Vector2d(7, 4));
}

TEST_CASE("compute minimum bounding, edge case") {
  std::vector<bh::Body> nodes;
  nodes.push_back({{1, 0}, 0});  // bottom-right body
  nodes.push_back({{1, 0}, 0});  // top-left body
  auto box = bh::compute_minimum_bounding_box(nodes);
  REQUIRE(box.min() == Vector2d(1, 0));
  REQUIRE(box.max() == Vector2d(1, 0));
}

TEST_CASE("compute square bounding box") {
  std::vector<bh::Body> nodes;
  nodes.push_back({{2.5, 4.0625}, 0});     // top-right node
  nodes.push_back({{9.375, 10.9375}, 0});  // bottom-left node
  auto box = bh::compute_square_bounding_box(nodes);
  REQUIRE(box.min() == Vector2d(2, 4));
  REQUIRE(box.max() == Vector2d(10, 12));
}
