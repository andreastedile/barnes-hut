#include "quadtree_deserialization.h"

#include <catch2/catch_test_macros.hpp>

SCENARIO("Deserialize a quadtree with a single node") {
  GIVEN("An empty quadtree") {
    auto quadtree = bh::mpi::Node{bh::mpi::Node::Leaf{}, 0, 0, 10, 10};
    auto deserialized = bh::deserialize_quadtree_node(quadtree);

    REQUIRE(deserialized->n_nodes() == 1);
    REQUIRE(deserialized->bbox().min() == Eigen::Vector2d{0, 0});
    REQUIRE(deserialized->bbox().max() == Eigen::Vector2d{10, 10});
    const auto &data = std::get<bh::Node::Leaf>(deserialized->data());
    REQUIRE_FALSE(data.m_body);

    WHEN("A body is inserted") {
      quadtree.data.leaf.body = bh::mpi::Body{7.5, 7.5, 0, 0, 0.25};
      quadtree.data.leaf.has_value = true;
      deserialized = bh::deserialize_quadtree_node(quadtree);

      const auto &data = std::get<bh::Node::Leaf>(deserialized->data());
      REQUIRE(data.m_body);
    }
  }
}

// https://www.desmos.com/calculator/ybk7rumtbn?lang=it
TEST_CASE("Deserialize a quadtree with a fork") {
  const auto blue = bh::mpi::Body{2.5, 7.5, 0, 0, 0.25};
  const auto green = bh::mpi::Body{7.5, 7.5, 0, 0, 0.25};
  const auto orange = bh::mpi::Body{7.5, 2.5, 0, 0, 0.25};
  const auto red = bh::mpi::Body{2.5, 2.5, 0, 0, 0.25};
  const auto aggregate = bh::mpi::Node::Fork::AggregateBody{5, 5, 0, 0, 1};

  const auto nw = bh::mpi::Node{bh::mpi::Node::Leaf{blue}, 0, 5, 5, 10};
  const auto ne = bh::mpi::Node{bh::mpi::Node::Leaf{green}, 5, 5, 10, 10};
  const auto se = bh::mpi::Node{bh::mpi::Node::Leaf{orange}, 5, 0, 10, 5};
  const auto sw = bh::mpi::Node{bh::mpi::Node::Leaf{red}, 0, 0, 5, 5};
  int children[4] = {1, 2, 3, 4};
  const auto quadtree = bh::mpi::Node{bh::mpi::Node::Fork{children, 4, aggregate}, 0, 0, 10, 10};

  const auto deserialized = bh::deserialize_quadtree({quadtree, nw, ne, se, sw});

  REQUIRE(deserialized->bbox().min() == Eigen::Vector2d{0, 0});
  REQUIRE(deserialized->bbox().max() == Eigen::Vector2d{10, 10});
  REQUIRE(deserialized->n_nodes() == 5);
  REQUIRE(deserialized->center_of_mass() == Eigen::Vector2d{5, 5});
  REQUIRE(deserialized->total_mass() == 1);
  const auto &data = std::get<bh::Node::Fork>(deserialized->data());

  const auto &deserialized_nw = *data.m_children[bh::Node::NW];
  REQUIRE(deserialized_nw.bbox().min() == Eigen::Vector2d{0, 5});
  REQUIRE(deserialized_nw.bbox().max() == Eigen::Vector2d{5, 10});
  REQUIRE(deserialized_nw.n_nodes() == 1);
  REQUIRE(deserialized_nw.center_of_mass() == Eigen::Vector2d{2.5, 7.5});
  REQUIRE(deserialized_nw.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<bh::Node::Leaf>(deserialized_nw.data()));

  const auto &deserialized_ne = *data.m_children[bh::Node::NE];
  REQUIRE(deserialized_ne.bbox().min() == Eigen::Vector2d{5, 5});
  REQUIRE(deserialized_ne.bbox().max() == Eigen::Vector2d{10, 10});
  REQUIRE(deserialized_ne.n_nodes() == 1);
  REQUIRE(deserialized_ne.center_of_mass() == Eigen::Vector2d{7.5, 7.5});
  REQUIRE(deserialized_ne.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<bh::Node::Leaf>(deserialized_ne.data()));

  const auto &deserialized_se = *data.m_children[bh::Node::SE];
  REQUIRE(deserialized_se.bbox().min() == Eigen::Vector2d{5, 0});
  REQUIRE(deserialized_se.bbox().max() == Eigen::Vector2d{10, 5});
  REQUIRE(deserialized_se.n_nodes() == 1);
  REQUIRE(deserialized_se.center_of_mass() == Eigen::Vector2d{7.5, 2.5});
  REQUIRE(deserialized_se.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<bh::Node::Leaf>(deserialized_se.data()));

  const auto &deserialized_sw = *data.m_children[bh::Node::SW];
  REQUIRE(deserialized_sw.bbox().min() == Eigen::Vector2d{0, 0});
  REQUIRE(deserialized_sw.bbox().max() == Eigen::Vector2d{5, 5});
  REQUIRE(deserialized_sw.n_nodes() == 1);
  REQUIRE(deserialized_sw.center_of_mass() == Eigen::Vector2d{2.5, 2.5});
  REQUIRE(deserialized_sw.total_mass() == 0.25);
  REQUIRE(std::holds_alternative<bh::Node::Leaf>(deserialized_sw.data()));
}
