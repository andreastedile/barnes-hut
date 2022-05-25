#include <catch2/catch.hpp>
#include <eigen3/Eigen/Geometry>
#include <iostream>

#include "barnes_hut/node.h"

using namespace bh;
using Eigen::Vector2f;

TEST_CASE("eigen aligned box") {
  Eigen::AlignedBox2f box(Vector2f(0, 0), Vector2f(10, 10));
  Vector2f top_left = box.corner(box.TopLeft);
  Vector2f top_right = box.corner(box.TopRight);
  Vector2f bottom_right = box.corner(box.BottomRight);
  Vector2f bottom_left = box.corner(box.BottomLeft);
  REQUIRE(top_left == Vector2f(0, 10));
  REQUIRE(top_right == Vector2f(10, 10));
  REQUIRE(bottom_right == Vector2f(10, 0));
  REQUIRE(bottom_left == Vector2f(0, 0));
}

TEST_CASE("empty tree creation") {
  Node root({0, 0}, {10, 10});

  REQUIRE(root.top_left() == Vector2f{0, 10});
  REQUIRE(root.length() == 10);
  REQUIRE_NOTHROW(std::get<Empty>(root.data()));
  REQUIRE(root.center_of_mass() == Vector2f{0, 0});
  REQUIRE(root.total_mass() == 0);
}

TEST_CASE("invalid body placement") {
  Node root(Vector2f(0, 0), Vector2f(10, 10));
  REQUIRE_THROWS(root.insert({{-2, 12}, 1}));
}

TEST_CASE("body insertion") {
  Node root(Vector2f(0, 0), Vector2f(10, 10));  // Root's quadrant has depth 0

  // body n. 1
  root.insert({{2, 8}, 1});
  REQUIRE_NOTHROW(std::get<Body>(root.data()));
  const auto& body = std::get<Body>(root.data());
  REQUIRE(body.m_position == Vector2f{2, 8});
  REQUIRE(body.m_mass == 1);
  REQUIRE(root.center_of_mass() == Vector2f{2, 8});
  REQUIRE(root.total_mass() == 1);

  // body n. 2
  root.insert({{3, 8}, 1});
  REQUIRE_NOTHROW(std::get<Subquadrants>(root.data()));
  REQUIRE(root.center_of_mass() == Vector2f{2.5, 8});
  REQUIRE(root.total_mass() == 2);

  // root quadrant, depth 0
  REQUIRE(root.center_of_mass() == Vector2f{2.5, 8});
  REQUIRE(root.total_mass() == 2);

  // depth 1, nowrth-west subquadrant
  const auto& nw1 = std::get<Subquadrants>(root.data())[NW];
  REQUIRE(nw1->top_left() == Vector2f{0, 10});
  REQUIRE(nw1->length() == 5);
  REQUIRE(nw1->center_of_mass() == Vector2f{2.5, 8});
  // depth 1, north-east subquadrant
  const auto& ne1 = std::get<Subquadrants>(root.data())[NE];
  REQUIRE(ne1->top_left() == Vector2f{5, 10});
  REQUIRE(ne1->length() == 5);
  REQUIRE(ne1->center_of_mass() == Vector2f{0, 0});
  // depth 1, south-east subquadrant
  const auto& se1 = std::get<Subquadrants>(root.data())[SE];
  REQUIRE(se1->top_left() == Vector2f{5, 5});
  REQUIRE(se1->length() == 5);
  REQUIRE(se1->center_of_mass() == Vector2f{0, 0});
  // depth 1, south-weast subquadrant
  const auto& sw1 = std::get<Subquadrants>(root.data())[SW];
  REQUIRE(sw1->top_left() == Vector2f{0, 5});
  REQUIRE(sw1->length() == 5);
  REQUIRE(sw1->center_of_mass() == Vector2f{0, 0});

  // depth 2, nowrth-west subquadrant
  const auto& nw2 = std::get<Subquadrants>(nw1->data())[NW];
  REQUIRE(nw2->top_left() == Vector2f{0, 10});
  REQUIRE(nw2->length() == 2.5);
  REQUIRE(nw2->center_of_mass() == Vector2f{2, 8});
  REQUIRE(nw2->total_mass() == 1);
  const auto& nw2body = std::get<Body>(nw2->data());
  REQUIRE(nw2body.m_position == Vector2f{2, 8});
  REQUIRE(nw2body.m_mass == 1);
  // depth 2, north-east subquadrant
  const auto& ne2 = std::get<Subquadrants>(nw1->data())[NE];
  REQUIRE(ne2->top_left() == Vector2f{2.5, 10});
  REQUIRE(ne2->length() == 2.5);
  REQUIRE(ne2->center_of_mass() == Vector2f{3, 8});
  REQUIRE(ne2->total_mass() == 1);
  const auto& ne2body = std::get<Body>(ne2->data());
  REQUIRE(ne2body.m_position == Vector2f{3, 8});
  REQUIRE(ne2body.m_mass == 1);
  // depth 2, south-east subquadrant
  const auto& se2 = std::get<Subquadrants>(nw1->data())[SE];
  REQUIRE(se2->top_left() == Vector2f{2.5, 7.5});
  REQUIRE(se2->length() == 2.5);
  REQUIRE(se2->center_of_mass() == Vector2f{0, 0});
  REQUIRE_NOTHROW(std::get<Empty>(se2->data()));
  // depth 2, south-weast subquadrant
  const auto& sw2 = std::get<Subquadrants>(nw1->data())[SW];
  REQUIRE(sw2->top_left() == Vector2f{0, 7.5});
  REQUIRE(sw2->length() == 2.5);
  REQUIRE(sw2->center_of_mass() == Vector2f{0, 0});
  REQUIRE_NOTHROW(std::get<Empty>(sw2->data()));
}

TEST_CASE("adding coinciding bodies") {
  Node root(Vector2f(0, 0), Vector2f(10, 10));
  root.insert({{2, 2}, 1});
  root.insert({{2, 2}, 1});  // prints "bodies coincide"

  const auto& body = std::get<Body>(root.data());
  REQUIRE(body.m_position == Vector2f{2, 2});
  REQUIRE(body.m_mass == 2);
}

TEST_CASE("bodies on subquadrant boundaries") {
  const Vector2f top_left{0, 10};
  const float length = 10;

  Vector2f p1{2.5f, 5};
  Vector2f p2{7.5f, 5};
  Vector2f p3{5, 7.5f};
  Vector2f p4{5, 2.5f};

  REQUIRE(get_subquadrant(top_left, length, p1) == NW);
  REQUIRE(get_subquadrant(top_left, length, p2) == NE);
  REQUIRE(get_subquadrant(top_left, length, p3) == NE);
  REQUIRE(get_subquadrant(top_left, length, p4) == SE);
}
