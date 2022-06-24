#include <catch2/catch.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <limits>

#include "node.h"

using namespace bh;
using Eigen::Vector2d;
using Eigen::AlignedBox2d;

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
