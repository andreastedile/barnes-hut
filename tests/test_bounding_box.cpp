#include <eigen3/Eigen/Eigen>
#include <vector>

#include "body.h"
#include "bounding_box.h"
#include "catch2/catch.hpp"

using Eigen::Vector2d;

SCENARIO("Computing minimum bounding box, edge cases") {
  GIVEN("An empty vector of bodies") {
    auto bodies = std::vector<bh::Body>{};

    THEN("The minimum bounding box is a point centered at the origin") {
      auto bbox = bh::compute_minimum_bounding_box(bodies);

      REQUIRE(bbox.min() == Vector2d{0, 0});
      REQUIRE(bbox.max() == Vector2d{0, 0});
    }
  }

  GIVEN("A vector with a single body") {
    auto first = bh::Body{{0, 1}, 0.25};
    auto bodies = std::vector<bh::Body>{first};

    THEN("The minimum bounding box is a dimensionless point centered at the coordinates of that body") {
      auto bbox = bh::compute_minimum_bounding_box(bodies);

      REQUIRE(bbox.min() == first.m_position);
      REQUIRE(bbox.max() == first.m_position);
    }

    WHEN("A body with the same coordinates is added") {
      auto second = first;
      bodies.push_back(second);

      THEN("The minimum bounding box does not change") {
        auto bbox = bh::compute_minimum_bounding_box(bodies);

        REQUIRE(bbox.min() == first.m_position);
        REQUIRE(bbox.max() == first.m_position);
      }
    }

    WHEN("A body with different coordinates is added") {
      auto second = bh::Body{{1, 0}, 0.25};
      bodies.push_back(second);

      THEN("The minimum bounding box changes") {
        auto bbox = bh::compute_minimum_bounding_box(bodies);

        REQUIRE(bbox.min() == Vector2d{0, 0});
        REQUIRE(bbox.max() == Vector2d{1, 1});
      }
    }
  }
}

SCENARIO("Computing minimum bounding box, common cases") {
  GIVEN("A vector with three non-coinciding bodies, aligned along the y=x diagonal") {
    auto bodies = std::vector<bh::Body>{
        {{-1, -1}, 0},  // bottom-left body
        {{0, 0}, 0},    // centered body
        {{1, 1}, 0}     // top-right body
    };

    auto box = bh::compute_minimum_bounding_box(bodies);
    REQUIRE(box.min() == Vector2d(-1, -1));
    REQUIRE(box.max() == Vector2d(1, 1));
  }

  GIVEN("A vector with three non-coinciding bodies, aligned along the y=-x diagonal") {
    auto bodies = std::vector<bh::Body>{
        {{-1, 1}, 0},  // top-left body
        {{0, 0}, 0},   // centered body
        {{1, -1}, 0}   // bottom-right body
    };

    auto box = bh::compute_minimum_bounding_box(bodies);
    REQUIRE(box.min() == Vector2d(-1, -1));
    REQUIRE(box.max() == Vector2d(1, 1));
  }
}

TEST_CASE("compute square bounding box") {
  std::vector<bh::Body> nodes;
  nodes.push_back({{2.5, 4.0625}, 0});     // top-right node
  nodes.push_back({{9.375, 10.9375}, 0});  // bottom-left node
  auto box = bh::compute_square_bounding_box(nodes);
  REQUIRE(box.min() == Vector2d(2, 4));
  REQUIRE(box.max() == Vector2d(10, 12));
}
