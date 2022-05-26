#include <vector>

#include "barnes_hut/body.h"
#include "catch2/catch.hpp"

TEST_CASE("compute minimum bounding box") {
  std::vector<bh::Body> nodes;
  nodes.push_back({{7, 4}, 0});      // top-right node
  nodes.push_back({{2, 3}, 0});      // bottom-left node
  nodes.push_back({{4.5, 3.5}, 0});  // somewhat in the center
  AlignedBox2f box = bh::compute_minimum_bounding_box(nodes);
  REQUIRE(box.min() == Vector2f(2, 3));
  REQUIRE(box.max() == Vector2f(7, 4));
}

TEST_CASE("compute square bounding box") {
  std::vector<bh::Body> nodes;
  nodes.push_back({{2.5, 4.0625}, 0});     // top-right node
  nodes.push_back({{9.375, 10.9375}, 0});  // bottom-left node
  AlignedBox2f box = bh::compute_square_bounding_box(nodes);
  REQUIRE(box.min() == Vector2f(2, 4));
  REQUIRE(box.max() == Vector2f(10, 12));
}
