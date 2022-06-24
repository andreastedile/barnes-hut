#include <catch2/catch.hpp>
#include <eigen3/Eigen/Eigen>
#include <variant>

#include "deserialization.h"
#include "node.h"

using namespace bh;
using Eigen::Vector2d;

TEST_CASE("deserialize an empty quadtree") {
  mpi::Node node{mpi::Node::Leaf{}, 0, 0, 10, 0};

  auto quadtree = deserialize({node});

  REQUIRE(quadtree->n_nodes() == 1);
  REQUIRE(quadtree->bbox() .min()== Vector2d{0, 0});
  REQUIRE(quadtree->bbox() .max()== Vector2d{10, 0});
  REQUIRE(std::holds_alternative<Node::Leaf>(quadtree->data()));
  const auto &data = std::get<Node::Leaf>(quadtree->data());
  REQUIRE_FALSE(data.m_body.has_value());
}

TEST_CASE("deserialize a quadtree with single node") {
  mpi::Body body{1.3, 4.5, 0.25};
  mpi::Node::Leaf leaf{body};
  mpi::Node node{leaf, 0, 0, 10, 0};

  auto quadtree = deserialize({node});

  REQUIRE(quadtree->n_nodes() == 1);
  REQUIRE(quadtree->bbox() .min()== Vector2d{0, 0});
  REQUIRE(quadtree->bbox() .max()== Vector2d{10, 0});
  REQUIRE(std::holds_alternative<Node::Leaf>(quadtree->data()));
  const auto &data = std::get<Node::Leaf>(quadtree->data());
  REQUIRE(data.m_body.has_value());
  REQUIRE(data.m_body.value().m_position == Vector2d{1.3, 4.5});
  REQUIRE(data.m_body.value().m_mass == 0.25);
}

// https://www.desmos.com/calculator/ybk7rumtbn?lang=it
TEST_CASE("deserialize a fork") {
  mpi::Body nwBody{2.5, 7.5, 0.25};
  mpi::Node::Leaf nwLeaf{nwBody};
  mpi::Node nwNode{nwLeaf, 0, 5, 5, 10};

  mpi::Body neBody{7.5, 7.5, 0.25};
  mpi::Node::Leaf neLeaf{neBody};
  mpi::Node neNode{neLeaf, 5, 5, 10, 10};

  mpi::Body seBody{7.5, 2.5, 0.25};
  mpi::Node::Leaf seLeaf{seBody};
  mpi::Node seNode{seLeaf, 5, 0, 10, 5};

  mpi::Body swBody{2.5, 2.5, 0.25};
  mpi::Node::Leaf swLeaf{swBody};
  mpi::Node swNode{swLeaf, 0, 0, 5, 5};

  mpi::Node::Fork::AggregateBody aggregate_body{5, 5, 1};
  int children[4]{1, 2, 3, 4};
  mpi::Node::Fork fork{children, 4, aggregate_body};
  mpi::Node root{fork, 0, 0, 10, 10};

  auto quadtree = deserialize({root, nwNode, neNode, seNode, swNode});

  REQUIRE(quadtree->bbox() .min()== Vector2d{0, 0});
  REQUIRE(quadtree->bbox() .max()== Vector2d{10, 10});
  REQUIRE(quadtree->n_nodes() == 5);
  REQUIRE(quadtree->center_of_mass() == Vector2d{5,5});
  REQUIRE(quadtree->total_mass() == 1);
  REQUIRE(std::holds_alternative<Node::Fork>(quadtree->data()));
  const auto& data = std::get<Node::Fork>(quadtree->data());

  const auto &nw = *data.m_children[Node::NW];
  REQUIRE(nw.bbox().min() == Vector2d{0, 5});
  REQUIRE(nw.bbox().max() == Vector2d{5, 10});
  REQUIRE(nw.n_nodes() == 1);
  REQUIRE(nw.center_of_mass() == Vector2d{2.5, 7.5});
  REQUIRE(nw.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<Node::Leaf>(nw.data()));

  const auto &ne = *data.m_children[Node::NE];
  REQUIRE(ne.bbox().min() == Vector2d{5, 5});
  REQUIRE(ne.bbox().max() == Vector2d{10, 10});
  REQUIRE(ne.n_nodes() == 1);
  REQUIRE(ne.center_of_mass() == Vector2d{7.5, 7.5});
  REQUIRE(ne.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<Node::Leaf>(ne.data()));

  const auto &se = *data.m_children[Node::SE];
  REQUIRE(se.bbox().min() == Vector2d{5, 0});
  REQUIRE(se.bbox().max() == Vector2d{10, 5});
  REQUIRE(se.n_nodes() == 1);
  REQUIRE(se.center_of_mass() == Vector2d{7.5, 2.5});
  REQUIRE(se.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<Node::Leaf>(se.data()));

  const auto &sw = *data.m_children[Node::SW];
  REQUIRE(sw.bbox().min() == Vector2d{0, 0});
  REQUIRE(sw.bbox().max() == Vector2d{5, 5});
  REQUIRE(sw.n_nodes() == 1);
  REQUIRE(sw.center_of_mass() == Vector2d{2.5, 2.5});
  REQUIRE(sw.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<Node::Leaf>(sw.data()));
}
