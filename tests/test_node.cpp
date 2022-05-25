#include <vector>

#include "barnes_hut/node.h"
#include "catch2/catch.hpp"

TEST_CASE("compute minimum bounding box") {
  std::vector<bh::Body> nodes;
  nodes.push_back({{7, 4}, 0});      // top-right node
  nodes.push_back({{2, 3}, 0});      // bottom-left node
  nodes.push_back({{4.5, 3.5}, 0});  // somewhat in the center
  AlignedBox2f box = bh::compute_minimum_bounding_box(nodes);
  REQUIRE(box.min() == Vector2f(2, 1));
  REQUIRE(box.max() == Vector2f(7, 6));
}
