#include <catch2/catch.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <limits>
#include <memory>
#include <utility>
#include <variant>

#include "node.h"
#include "quadtree.h"

using namespace bh;
using Eigen::AlignedBox2d;
using Eigen::Vector2d;

TEST_CASE("empty tree creation") {
  Node root({0, 0}, {10, 10});

  REQUIRE(root.bbox().min() == Vector2d{0, 0});
  REQUIRE(root.bbox().max() == Vector2d{10, 10});
  REQUIRE_NOTHROW(std::get<Node::Leaf>(root.data()));
  REQUIRE_FALSE(std::get<Node::Leaf>(root.data()).m_body.has_value());
  REQUIRE(root.center_of_mass() == Vector2d{0, 0});
  REQUIRE(root.total_mass() == 0);
}

TEST_CASE("invalid body placement") {
  Node root(Vector2d(0, 0), Vector2d(10, 10));
  REQUIRE_THROWS(root.insert({{-2, 12}, 1}));
}

TEST_CASE("body insertion") {
  Node root(Vector2d(0, 0), Vector2d(10, 10));  // Root's quadrant has depth 0

  // body n. 1
  root.insert({{2, 8}, 1});
  REQUIRE_NOTHROW(std::get<Node::Leaf>(root.data()));
  const auto& body = *std::get<Node::Leaf>(root.data()).m_body;
  REQUIRE(body.m_position == Vector2d{2, 8});
  REQUIRE(body.m_mass == 1);
  REQUIRE(root.center_of_mass() == Vector2d{2, 8});
  REQUIRE(root.total_mass() == 1);

  // body n. 2
  root.insert({{3, 8}, 1});
  REQUIRE_NOTHROW(std::get<Node::Fork>(root.data()));
  REQUIRE(root.center_of_mass() == Vector2d{2.5, 8});
  REQUIRE(root.total_mass() == 2);

  // root quadrant, depth 0
  REQUIRE(root.center_of_mass() == Vector2d{2.5, 8});
  REQUIRE(root.total_mass() == 2);

  // depth 1, nowrth-west subquadrant
  const auto& nw1 = *std::get<Node::Fork>(root.data()).m_children[Node::NW];
  REQUIRE(nw1.bbox().min() == Vector2d{0, 5});
  REQUIRE(nw1.bbox().max() == Vector2d{5, 10});
  REQUIRE(nw1.center_of_mass() == Vector2d{2.5, 8});
  // depth 1, north-east subquadrant
  const auto& ne1 = *std::get<Node::Fork>(root.data()).m_children[Node::NE];
  REQUIRE(ne1.bbox().min() == Vector2d{5, 5});
  REQUIRE(ne1.bbox().max() == Vector2d{10, 10});
  REQUIRE(ne1.center_of_mass() == Vector2d{0, 0});
  // depth 1, south-east subquadrant
  const auto& se1 = *std::get<Node::Fork>(root.data()).m_children[Node::SE];
  REQUIRE(se1.bbox().min() == Vector2d{5, 0});
  REQUIRE(se1.bbox().max() == Vector2d{10, 5});
  REQUIRE(se1.center_of_mass() == Vector2d{0, 0});
  // depth 1, south-weast subquadrant
  const auto& sw1 = *std::get<Node::Fork>(root.data()).m_children[Node::SW];
  REQUIRE(sw1.bbox().min() == Vector2d{0, 0});
  REQUIRE(sw1.bbox().max() == Vector2d{5, 5});
  REQUIRE(sw1.center_of_mass() == Vector2d{0, 0});

  // depth 2, nowrth-west subquadrant
  const auto& nw2 = *std::get<Node::Fork>(nw1.data()).m_children[Node::NW];
  REQUIRE(nw2.bbox().min() == Vector2d{0, 7.5});
  REQUIRE(nw2.bbox().max() == Vector2d{2.5, 10});
  REQUIRE(nw2.center_of_mass() == Vector2d{2, 8});
  REQUIRE(nw2.total_mass() == 1);
  const auto& nw2body = *std::get<Node::Leaf>(nw2.data()).m_body;
  REQUIRE(nw2body.m_position == Vector2d{2, 8});
  REQUIRE(nw2body.m_mass == 1);
  // depth 2, north-east subquadrant
  const auto& ne2 = *std::get<Node::Fork>(nw1.data()).m_children[Node::NE];
  REQUIRE(ne2.bbox().min() == Vector2d{2.5, 7.5});
  REQUIRE(ne2.bbox().max() == Vector2d{5, 10});
  REQUIRE(ne2.center_of_mass() == Vector2d{3, 8});
  REQUIRE(ne2.total_mass() == 1);
  const auto& ne2body = *std::get<Node::Leaf>(ne2.data()).m_body;
  REQUIRE(ne2body.m_position == Vector2d{3, 8});
  REQUIRE(ne2body.m_mass == 1);
  // depth 2, south-east subquadrant
  const auto& se2 = *std::get<Node::Fork>(nw1.data()).m_children[Node::SE];
  REQUIRE(se2.bbox().min() == Vector2d{2.5, 5});
  REQUIRE(se2.bbox().max() == Vector2d{5, 7.5});
  REQUIRE(se2.center_of_mass() == Vector2d{0, 0});
  REQUIRE_NOTHROW(std::get<Node::Leaf>(se2.data()));
  // depth 2, south-weast subquadrant
  const auto& sw2 = *std::get<Node::Fork>(nw1.data()).m_children[Node::SW];
  REQUIRE(sw2.bbox().min() == Vector2d{0, 5});
  REQUIRE(sw2.bbox().max() == Vector2d{2.5, 7.5});
  REQUIRE(sw2.center_of_mass() == Vector2d{0, 0});
  REQUIRE_NOTHROW(std::get<Node::Leaf>(sw2.data()));
}

TEST_CASE("adding coinciding bodies") {
  Node root(Vector2d(0, 0), Vector2d(10, 10));
  root.insert({{2, 2}, 1});
  root.insert({{2, 2}, 1});  // prints "bodies coincide"

  const auto& body = *std::get<Node::Leaf>(root.data()).m_body;
  REQUIRE(body.m_position == Vector2d{2, 2});
  REQUIRE(body.m_mass == 2);
}

TEST_CASE("merge four nodes with empty leaves") {
  auto nw = std::make_unique<Node>(Vector2d{0, 1}, Vector2d{1, 2});
  auto ne = std::make_unique<Node>(Vector2d{1, 1}, Vector2d{2, 2});
  auto se = std::make_unique<Node>(Vector2d{1, 0}, Vector2d{2, 1});
  auto sw = std::make_unique<Node>(Vector2d{0, 0}, Vector2d{1, 1});
  auto qt = merge_quadtrees(std::move(nw), std::move(ne), std::move(se), std::move(sw));
  REQUIRE(qt->bbox().min() == Vector2d{0, 0});
  REQUIRE(qt->bbox().max() == Vector2d{2, 2});
  const auto& data = qt->data();
  // both are equivalent
  REQUIRE(qt->n_nodes() == 1);
  const auto& leaf = std::get<Node::Leaf>(data);
  const auto body = leaf.m_body;
  REQUIRE_FALSE(body);
}

TEST_CASE("merge four nodes with just one body") {
  auto nw = std::make_unique<Node>(Vector2d{0, 1}, Vector2d{1, 2});
  nw->insert({Vector2d{0.5, 1.5}, 0.25});
  auto ne = std::make_unique<Node>(Vector2d{1, 1}, Vector2d{2, 2});
  auto se = std::make_unique<Node>(Vector2d{1, 0}, Vector2d{2, 1});
  auto sw = std::make_unique<Node>(Vector2d{0, 0}, Vector2d{1, 1});
  auto qt = merge_quadtrees(std::move(nw), std::move(ne), std::move(se), std::move(sw));
  REQUIRE(qt->bbox().min() == Vector2d{0, 0});
  REQUIRE(qt->bbox().max() == Vector2d{2, 2});
  const auto& data = qt->data();
  // both are equivalent
  REQUIRE(qt->n_nodes() == 1);
  const auto& leaf = std::get<Node::Leaf>(data);
  REQUIRE(leaf.m_body);
  REQUIRE(leaf.m_body.value().m_position == Vector2d{0.5, 1.5});
  REQUIRE(leaf.m_body.value().m_mass == 0.25);
}

TEST_CASE("merge four nodes with two bodies") {
  auto nw = std::make_unique<Node>(Vector2d{0, 1}, Vector2d{1, 2});
  nw->insert({Vector2d{0.5, 1.5}, 0.25});
  auto ne = std::make_unique<Node>(Vector2d{1, 1}, Vector2d{2, 2});
  ne->insert({Vector2d{1.5, 1.5}, 0.25});
  auto se = std::make_unique<Node>(Vector2d{1, 0}, Vector2d{2, 1});
  auto sw = std::make_unique<Node>(Vector2d{0, 0}, Vector2d{1, 1});
  auto qt = merge_quadtrees(std::move(nw), std::move(ne), std::move(se), std::move(sw));
  REQUIRE(qt->bbox().min() == Vector2d{0, 0});
  REQUIRE(qt->bbox().max() == Vector2d{2, 2});
  const auto& data = qt->data();
  // both are equivalent
  REQUIRE(qt->n_nodes() == 5);

  const auto& fork = std::get<Node::Fork>(data);
  const auto& fork_nw = *fork.m_children[Node::NW];
  const auto& fork_ne = *fork.m_children[Node::NE];
  const auto& fork_se = *fork.m_children[Node::SE];
  const auto& fork_sw = *fork.m_children[Node::SW];

  const auto& leaf_nw = std::get<Node::Leaf>(fork_nw.data());
  REQUIRE(leaf_nw.m_body);
  REQUIRE(leaf_nw.m_body.value().m_position == Vector2d{0.5, 1.5});
  REQUIRE(leaf_nw.m_body.value().m_mass == 0.25);

  const auto& leaf_ne = std::get<Node::Leaf>(fork_ne.data());
  REQUIRE(leaf_ne.m_body);
  REQUIRE(leaf_ne.m_body.value().m_position == Vector2d{1.5, 1.5});
  REQUIRE(leaf_ne.m_body.value().m_mass == 0.25);

  const auto& leaf_se = std::get<Node::Leaf>(fork_se.data());
  REQUIRE_FALSE(leaf_se.m_body);

  const auto& leaf_sw = std::get<Node::Leaf>(fork_sw.data());
  REQUIRE_FALSE(leaf_sw.m_body);
}

TEST_CASE("reconstruct 8x8 quadtree matrix") {
  QuadtreeGrid matrix(8);
  for (int i = 0; i < 8; i++) {
    matrix[i].resize(8);
  }

  // A
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix[i][j] = std::make_unique<Node>(Vector2d(j, 7 - i), Vector2d(j + 1, 8 - i));
    }
  }
  // B
  for (int i = 4; i < 8; i++) {
    for (int j = 4; j < 8; j++) {
      matrix[i][j] = std::make_unique<Node>(Vector2d(j, 7 - i), Vector2d(j + 1, 8 - i));
    }
  }
  // C
  for (int i = 0; i < 2; i++) {
    for (int j = 4; j < 6; j++) {
      matrix[i][j] = std::make_unique<Node>(Vector2d(j, 7 - i), Vector2d(j + 1, 8 - i));
    }
  }
  // D
  for (int i = 2; i < 4; i++) {
    for (int j = 6; j < 8; j++) {
      matrix[i][j] = std::make_unique<Node>(Vector2d(j, 7 - i), Vector2d(j + 1, 8 - i));
    }
  }
  // E
  for (int i = 4; i < 6; i++) {
    for (int j = 0; j < 2; j++) {
      matrix[i][j] = std::make_unique<Node>(Vector2d(j, 7 - i), Vector2d(j + 1, 8 - i));
    }
  }
  // F
  for (int i = 6; i < 8; i++) {
    for (int j = 2; j < 4; j++) {
      matrix[i][j] = std::make_unique<Node>(Vector2d(j, 7 - i), Vector2d(j + 1, 8 - i));
    }
  }
  // G
  matrix[0][6] = std::make_unique<Node>(Vector2d(6, 7), Vector2d(7, 8));
  // H
  matrix[1][7] = std::make_unique<Node>(Vector2d(7, 6), Vector2d(8, 7));
  // I
  matrix[2][4] = std::make_unique<Node>(Vector2d(4, 5), Vector2d(5, 6));
  // J
  matrix[2][5] = std::make_unique<Node>(Vector2d(5, 5), Vector2d(6, 6));
  // K
  matrix[3][5] = std::make_unique<Node>(Vector2d(5, 4), Vector2d(6, 5));
  // L
  matrix[4][2] = std::make_unique<Node>(Vector2d(2, 3), Vector2d(3, 4));
  // M
  matrix[5][2] = std::make_unique<Node>(Vector2d(2, 2), Vector2d(3, 3));
  // N
  matrix[5][3] = std::make_unique<Node>(Vector2d(3, 2), Vector2d(4, 3));
  // O
  matrix[6][0] = std::make_unique<Node>(Vector2d(0, 1), Vector2d(1, 2));
  // P
  matrix[7][1] = std::make_unique<Node>(Vector2d(1, 0), Vector2d(2, 1));
  // Q
  matrix[0][7] = std::make_unique<Node>(Vector2d(7, 7), Vector2d(8, 8));
  matrix[0][7]->insert({{7.5, 7.5}, 0.25});
  // R
  matrix[1][6] = std::make_unique<Node>(Vector2d(6, 6), Vector2d(7, 7));
  matrix[1][6]->insert({{6.5, 6.5}, 0.25});
  // S
  matrix[3][4] = std::make_unique<Node>(Vector2d(4, 4), Vector2d(5, 5));
  matrix[3][4]->insert({{4.5, 4.5}, 0.25});
  // T
  matrix[4][3] = std::make_unique<Node>(Vector2d(3, 3), Vector2d(4, 4));
  matrix[4][3]->insert({{3.5, 3.5}, 0.25});
  // U
  matrix[6][1] = std::make_unique<Node>(Vector2d(1, 1), Vector2d(2, 2));
  matrix[6][1]->insert({{1.5, 1.5}, 0.25});
  // V
  matrix[7][0] = std::make_unique<Node>(Vector2d(0, 0), Vector2d(1, 1));
  matrix[7][0]->insert({{0.5, 0.5}, 0.25});

  auto quadtree = reconstruct_quadtree(matrix);

  REQUIRE(quadtree->bbox().min() == Vector2d{0, 0});
  REQUIRE(quadtree->bbox().max() == Vector2d{8, 8});
  REQUIRE(quadtree->center_of_mass() == Vector2d{4, 4});
  REQUIRE(quadtree->total_mass() == 1.5);
  REQUIRE(quadtree->n_nodes() == 21);
  const auto& fork_0 = std::get<Node::Fork>(quadtree->data());

  const auto& A = *fork_0.m_children[Node::NW];
  auto min = A.bbox().min();
  auto max = A.bbox().max();
  REQUIRE(A.bbox().min() == Vector2d{0, 4});
  REQUIRE(A.bbox().max() == Vector2d{4, 8});
  REQUIRE(A.total_mass() == 0);
  REQUIRE(A.n_nodes() == 1);

  const auto& B = *fork_0.m_children[Node::SE];
  REQUIRE(B.bbox().min() == Vector2d{4, 0});
  REQUIRE(B.bbox().max() == Vector2d{8, 4});
  REQUIRE(B.center_of_mass() == Vector2d{0, 0});
  REQUIRE(B.total_mass() == 0);
  REQUIRE(B.n_nodes() == 1);

  const auto& fork0_ne = *fork_0.m_children[Node::NE];
  REQUIRE(fork0_ne.bbox().min() == Vector2d{4, 4});
  REQUIRE(fork0_ne.bbox().max() == Vector2d{8, 8});
  REQUIRE(fork0_ne.center_of_mass().x() == Approx(6.16666666667));
  REQUIRE(fork0_ne.center_of_mass().y() == Approx(6.16666666667));
  REQUIRE(fork0_ne.total_mass() == 0.25 * 3);
  REQUIRE(fork0_ne.n_nodes() == 9);

  const auto& fork0_sw = *fork_0.m_children[Node::SW];
  REQUIRE(fork0_sw.bbox().min() == Vector2d{0, 0});
  REQUIRE(fork0_sw.bbox().max() == Vector2d{4, 4});
  REQUIRE(fork0_sw.center_of_mass().x() == Approx(1.83333333333));
  REQUIRE(fork0_sw.center_of_mass().y() == Approx(1.83333333333));
  REQUIRE(fork0_sw.total_mass() == 0.25 * 3);
  REQUIRE(fork0_sw.n_nodes() == 9);

  const auto& fork1_ne = std::get<Node::Fork>(fork0_ne.data());
  const auto& leaf1_ne = *fork1_ne.m_children[Node::SW];
  REQUIRE(leaf1_ne.bbox().min() == Vector2d{4, 4});
  REQUIRE(leaf1_ne.bbox().max() == Vector2d{6, 6});
  REQUIRE(leaf1_ne.center_of_mass() == Vector2d{4.5, 4.5});
  REQUIRE(leaf1_ne.total_mass() == 0.25);
  REQUIRE(leaf1_ne.n_nodes() == 1);

  const auto& node1_sw = *fork_0.m_children[Node::SW];
  const auto& fork1_sw = std::get<Node::Fork>(node1_sw.data());
  const auto& node2_sw = *fork1_sw.m_children[Node::SW];
  REQUIRE(node2_sw.bbox().min() == Vector2d{0, 0});
  REQUIRE(node2_sw.bbox().max() == Vector2d{2, 2});
  REQUIRE(node2_sw.center_of_mass() == Vector2d{1, 1});
  REQUIRE(node2_sw.total_mass() == 2 * 0.25);
  REQUIRE(node2_sw.n_nodes() == 5);

  const auto& fork2_sw = std::get<Node::Fork>(node2_sw.data());
  const auto& node3_nw = *fork2_sw.m_children[Node::NW];
  REQUIRE(node3_nw.bbox().min() == Vector2d{0, 1});
  REQUIRE(node3_nw.bbox().max() == Vector2d{1, 2});
  REQUIRE(node3_nw.center_of_mass() == Vector2d{0, 0});
  REQUIRE(node3_nw.total_mass() == 0);
  REQUIRE(node3_nw.n_nodes() == 1);
  const auto& node3_ne = *fork2_sw.m_children[Node::NE];
  REQUIRE(node3_ne.bbox().min() == Vector2d{1, 1});
  REQUIRE(node3_ne.bbox().max() == Vector2d{2, 2});
  REQUIRE(node3_ne.center_of_mass() == Vector2d{1.5, 1.5});
  REQUIRE(node3_ne.total_mass() == 0.25);
  REQUIRE(node3_ne.n_nodes() == 1);
  const auto& node3_se = *fork2_sw.m_children[Node::SE];
  REQUIRE(node3_se.bbox().min() == Vector2d{1, 0});
  REQUIRE(node3_se.bbox().max() == Vector2d{2, 1});
  REQUIRE(node3_se.center_of_mass() == Vector2d{0, 0});
  REQUIRE(node3_se.total_mass() == 0);
  REQUIRE(node3_se.n_nodes() == 1);
  const auto& node3_sw = *fork2_sw.m_children[Node::SW];
  REQUIRE(node3_sw.bbox().min() == Vector2d{0, 0});
  REQUIRE(node3_sw.bbox().max() == Vector2d{1, 1});
  REQUIRE(node3_sw.center_of_mass() == Vector2d{0.5, 0.5});
  REQUIRE(node3_sw.total_mass() == 0.25);
  REQUIRE(node3_sw.n_nodes() == 1);
}
