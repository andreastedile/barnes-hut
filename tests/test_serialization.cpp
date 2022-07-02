#include <catch2/catch.hpp>

#include "node.h"
#include "serialization.h"

using namespace bh;

TEST_CASE("serialize_quadtree an empty quadtree") {
  Node quadtree({0, 0}, {10, 10});
  auto serialized = serialize(quadtree);
  REQUIRE(serialized.size() == 1);
  const auto& node = serialized[0];
  REQUIRE(node.bottom_left_x == 0);
  REQUIRE(node.bottom_left_y == 0);
  REQUIRE(node.top_right_x == 10);
  REQUIRE(node.top_right_y == 10);
  REQUIRE(node.type == bh::mpi::Node::LeafType);
  REQUIRE_FALSE(node.data.leaf.has_value);
}

TEST_CASE("serialize_quadtree a quadtree with single node") {
  Node quadtree({0, 0}, {10, 10});
  quadtree.insert({{7.5, 7.5}, 0.25});
  auto serialized = serialize(quadtree);
  REQUIRE(serialized.size() == 1);
  const auto& node = serialized[0];
  REQUIRE(node.bottom_left_x == 0);
  REQUIRE(node.bottom_left_y == 0);
  REQUIRE(node.top_right_x == 10);
  REQUIRE(node.top_right_y == 10);
  REQUIRE(node.type == bh::mpi::Node::LeafType);
  REQUIRE(node.data.leaf.has_value);
  REQUIRE(node.data.leaf.body.position_x == 7.5);
  REQUIRE(node.data.leaf.body.position_y == 7.5);
  REQUIRE(node.data.leaf.body.mass == 0.25);
}

// https://www.desmos.com/calculator/wpyi6tikb2?lang=it
TEST_CASE("serialize_quadtree complex quadtree") {
  Node quadtree({0, 0}, {10, 10});
  quadtree.insert({{0, 0}, 0.25});
  quadtree.insert({{2, 2}, 0.25});
  quadtree.insert({{4, 4}, 0.25});
  quadtree.insert({{6, 6}, 0.25});
  quadtree.insert({{8, 8}, 0.25});
  quadtree.insert({{10, 10}, 0.25});

  auto serialized = serialize(quadtree);
  REQUIRE(serialized.size() == 21);

  REQUIRE(serialized[0].type == bh::mpi::Node::ForkType);
  REQUIRE(serialized[1].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[2].type == bh::mpi::Node::ForkType);
  REQUIRE(serialized[3].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[4].type == bh::mpi::Node::ForkType);
  REQUIRE(serialized[5].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[6].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[7].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[8].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[9].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[10].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[11].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[12].type == bh::mpi::Node::ForkType);
  REQUIRE(serialized[13].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[14].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[15].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[16].type == bh::mpi::Node::ForkType);
  REQUIRE(serialized[17].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[18].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[19].type == bh::mpi::Node::LeafType);
  REQUIRE(serialized[20].type == bh::mpi::Node::LeafType);

  REQUIRE(serialized[0].data.fork.children[bh::Node::NW] == 1);
  REQUIRE(serialized[0].data.fork.children[bh::Node::NE] == 2);
  REQUIRE(serialized[0].data.fork.children[bh::Node::SE] == 11);
  REQUIRE(serialized[0].data.fork.children[bh::Node::SW] == 12);

  REQUIRE(serialized[2].data.fork.children[bh::Node::NW] == 3);
  REQUIRE(serialized[2].data.fork.children[bh::Node::NE] == 4);
  REQUIRE(serialized[2].data.fork.children[bh::Node::SE] == 9);
  REQUIRE(serialized[2].data.fork.children[bh::Node::SW] == 10);

  REQUIRE(serialized[12].data.fork.children[bh::Node::NW] == 13);
  REQUIRE(serialized[12].data.fork.children[bh::Node::NE] == 14);
  REQUIRE(serialized[12].data.fork.children[bh::Node::SE] == 15);
  REQUIRE(serialized[12].data.fork.children[bh::Node::SW] == 16);

  REQUIRE(serialized[4].data.fork.children[bh::Node::NW] == 5);
  REQUIRE(serialized[4].data.fork.children[bh::Node::NE] == 6);
  REQUIRE(serialized[4].data.fork.children[bh::Node::SE] == 7);
  REQUIRE(serialized[4].data.fork.children[bh::Node::SW] == 8);

  REQUIRE(serialized[16].data.fork.children[bh::Node::NW] == 17);
  REQUIRE(serialized[16].data.fork.children[bh::Node::NE] == 18);
  REQUIRE(serialized[16].data.fork.children[bh::Node::SE] == 19);
  REQUIRE(serialized[16].data.fork.children[bh::Node::SW] == 20);
}
