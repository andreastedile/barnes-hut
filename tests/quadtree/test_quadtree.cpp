#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "quadtree.h"

// https://www.desmos.com/calculator/ztgyolrttb?lang=it
SCENARIO("Construct a simple quadtree") {
  GIVEN("An empty quadtree") {
    auto lv0 = bh::Node{{0, 0}, {10, 10}};

    THEN("lv0 (quadtree's root node) is an empty leaf") {
      REQUIRE(lv0.bbox().min() == Eigen::Vector2d{0, 0});
      REQUIRE(lv0.bbox().max() == Eigen::Vector2d{10, 10});
      REQUIRE(lv0.n_nodes() == 1);
      REQUIRE(lv0.center_of_mass() == Eigen::Vector2d{0, 0});
      REQUIRE(lv0.total_mass() == 0);
      const auto& data = std::get<bh::Node::Leaf>(lv0.data());
      REQUIRE_FALSE(data.m_body);
    }

    WHEN("A body outside of the bounding box is inserted") {
      const auto body_outside = bh::Body{{-2, 12}, 1};

      THEN("An error is thrown") {
        REQUIRE_THROWS(lv0.insert(body_outside));
      }
    }

    WHEN("The first, blue body is inserted") {
      const auto blue_body = bh::Body{{2, 8}, 1};
      lv0.insert(blue_body);

      THEN("lv0 (quadtree's root node) now contains the blue body") {
        const auto& lv0_data = std::get<bh::Node::Leaf>(lv0.data());

        REQUIRE(lv0.center_of_mass() == blue_body.m_position);
        REQUIRE(lv0.total_mass() == blue_body.m_mass);
        REQUIRE(lv0_data.m_body);
      }

      WHEN("The second, green body is inserted") {
        const auto green_body = bh::Body{{3, 8}, 1};
        lv0.insert(green_body);

        THEN("lv0 (quadtree's root node) node becomes a fork") {
          REQUIRE(lv0.n_nodes() == 9);
          REQUIRE(lv0.center_of_mass() == (blue_body.m_position + green_body.m_position) / 2);
          REQUIRE(lv0.total_mass() == blue_body.m_mass + green_body.m_mass);

          const auto& lv0_data = std::get<bh::Node::Fork>(lv0.data());

          THEN("The lv1-NW becomes a leaf containing the blue body") {
            const auto& lv1_nw = *lv0_data.m_children[bh::Node::NW];

            REQUIRE(lv1_nw.bbox().min() == Eigen::Vector2d{0, 5});
            REQUIRE(lv1_nw.bbox().max() == Eigen::Vector2d{5, 10});
            REQUIRE(lv1_nw.n_nodes() == 5);
            REQUIRE(lv1_nw.center_of_mass() == (blue_body.m_position + green_body.m_position) / 2);
            REQUIRE(lv1_nw.total_mass() == blue_body.m_mass + green_body.m_mass);

            const auto& lv1_nw_data = std::get<bh::Node::Fork>(lv1_nw.data());

            THEN("The lv2-NE becomes a leaf containing the blue body") {
              const auto& lv2_nw = *lv1_nw_data.m_children[bh::Node::NW];

              REQUIRE(lv2_nw.bbox().min() == Eigen::Vector2d{0, 7.5});
              REQUIRE(lv2_nw.bbox().max() == Eigen::Vector2d{2.5, 10});
              REQUIRE(lv2_nw.n_nodes() == 1);
              REQUIRE(lv2_nw.center_of_mass() == blue_body.m_position);
              REQUIRE(lv2_nw.total_mass() == blue_body.m_mass);

              const auto& lv2_nw_data = std::get<bh::Node::Leaf>(lv2_nw.data());
              REQUIRE(lv2_nw_data.m_body);
            }

            THEN("The lv2-NE becomes a leaf containing the green body") {
              const auto& lv2_ne = *lv1_nw_data.m_children[bh::Node::NE];

              REQUIRE(lv2_ne.bbox().min() == Eigen::Vector2d{2.5, 7.5});
              REQUIRE(lv2_ne.bbox().max() == Eigen::Vector2d{5, 10});
              REQUIRE(lv2_ne.n_nodes() == 1);
              REQUIRE(lv2_ne.center_of_mass() == green_body.m_position);
              REQUIRE(lv2_ne.total_mass() == green_body.m_mass);

              const auto& lv2_nw_data = std::get<bh::Node::Leaf>(lv2_ne.data());
              REQUIRE(lv2_nw_data.m_body);
            }

            THEN("The lv2-SE becomes an empty leaf") {
              const auto& lv2_se = *lv1_nw_data.m_children[bh::Node::SE];

              REQUIRE(lv2_se.bbox().min() == Eigen::Vector2d{2.5, 5});
              REQUIRE(lv2_se.bbox().max() == Eigen::Vector2d{5, 7.5});
              REQUIRE(lv2_se.n_nodes() == 1);
              REQUIRE(lv2_se.center_of_mass() == Eigen::Vector2d{0, 0});
              REQUIRE(lv2_se.total_mass() == 0);
            }

            THEN("The lv2-SW becomes an empty leaf") {
              const auto& lv2_sw = *lv1_nw_data.m_children[bh::Node::SW];

              REQUIRE(lv2_sw.bbox().min() == Eigen::Vector2d{0, 5});
              REQUIRE(lv2_sw.bbox().max() == Eigen::Vector2d{2.5, 7.5});
              REQUIRE(lv2_sw.n_nodes() == 1);
              REQUIRE(lv2_sw.center_of_mass() == Eigen::Vector2d{0, 0});
              REQUIRE(lv2_sw.total_mass() == 0);
            }
          }

          THEN("The lv1-NE becomes an empty leaf") {
            const auto& lv1_ne = *std::get<bh::Node::Fork>(lv0.data()).m_children[bh::Node::NE];

            REQUIRE(lv1_ne.bbox().min() == Eigen::Vector2d{5, 5});
            REQUIRE(lv1_ne.bbox().max() == Eigen::Vector2d{10, 10});
            REQUIRE(lv1_ne.n_nodes() == 1);
            REQUIRE(lv1_ne.center_of_mass() == Eigen::Vector2d{0, 0});
            REQUIRE(lv1_ne.total_mass() == 0);
          }

          THEN("The lv1-SE node becomes an empty leaf") {
            const auto& lv1_se = *std::get<bh::Node::Fork>(lv0.data()).m_children[bh::Node::SE];

            REQUIRE(lv1_se.bbox().min() == Eigen::Vector2d{5, 0});
            REQUIRE(lv1_se.bbox().max() == Eigen::Vector2d{10, 5});
            REQUIRE(lv1_se.n_nodes() == 1);
            REQUIRE(lv1_se.center_of_mass() == Eigen::Vector2d{0, 0});
            REQUIRE(lv1_se.total_mass() == 0);
          }

          THEN("The lv1-SW node becomes an empty leaf") {
            const auto& lv1_sw = *std::get<bh::Node::Fork>(lv0.data()).m_children[bh::Node::SW];

            REQUIRE(lv1_sw.bbox().min() == Eigen::Vector2d{0, 0});
            REQUIRE(lv1_sw.bbox().max() == Eigen::Vector2d{5, 5});
            REQUIRE(lv1_sw.n_nodes() == 1);
            REQUIRE(lv1_sw.center_of_mass() == Eigen::Vector2d{0, 0});
            REQUIRE(lv1_sw.total_mass() == 0);
          }
        }
      }

      WHEN("A second body is added, which coincides with the first, blue node") {
        const auto& coinciding_body = blue_body;
        lv0.insert(coinciding_body);

        THEN("The mass of the blue node increases by the amount of the second body") {
          REQUIRE(lv0.total_mass() == blue_body.m_mass * 2);
        }
      }
    }
  }
}

TEST_CASE("Merge four quadtree nodes with empty leaves in a single quadtree node") {
  auto nw = std::make_unique<bh::Node>(Eigen::Vector2d{0, 1}, Eigen::Vector2d{1, 2});
  auto ne = std::make_unique<bh::Node>(Eigen::Vector2d{1, 1}, Eigen::Vector2d{2, 2});
  auto se = std::make_unique<bh::Node>(Eigen::Vector2d{1, 0}, Eigen::Vector2d{2, 1});
  auto sw = std::make_unique<bh::Node>(Eigen::Vector2d{0, 0}, Eigen::Vector2d{1, 1});

  const auto merged = merge_quadtrees(std::move(nw), std::move(ne), std::move(se), std::move(sw));
  REQUIRE(merged->bbox().min() == Eigen::Vector2d{0, 0});
  REQUIRE(merged->bbox().max() == Eigen::Vector2d{2, 2});
  REQUIRE(merged->n_nodes() == 1);
}

TEST_CASE("Merge four quadtree nodes, one with a single body, in a single quadtree node") {
  auto nw = std::make_unique<bh::Node>(Eigen::Vector2d{0, 1}, Eigen::Vector2d{1, 2});
  auto ne = std::make_unique<bh::Node>(Eigen::Vector2d{1, 1}, Eigen::Vector2d{2, 2});
  auto se = std::make_unique<bh::Node>(Eigen::Vector2d{1, 0}, Eigen::Vector2d{2, 1});
  auto sw = std::make_unique<bh::Node>(Eigen::Vector2d{0, 0}, Eigen::Vector2d{1, 1});

  const auto body = bh::Body{{0.5, 1.5}, 0.25};
  nw->insert(body);

  const auto merged = merge_quadtrees(std::move(nw), std::move(ne), std::move(se), std::move(sw));

  REQUIRE(merged->bbox().min() == Eigen::Vector2d{0, 0});
  REQUIRE(merged->bbox().max() == Eigen::Vector2d{2, 2});
  REQUIRE(merged->n_nodes() == 1);
  REQUIRE(merged->center_of_mass() == body.m_position);
  REQUIRE(merged->total_mass() == body.m_mass);

  const auto& data = std::get<bh::Node::Leaf>(merged->data());
  REQUIRE(data.m_body);
  REQUIRE(data.m_body->m_position == body.m_position);
  REQUIRE(data.m_body->m_mass == body.m_mass);
}

SCENARIO("Merge four quadtree nodes, two with a single body, in a single quadtree node") {
  auto nw = std::make_unique<bh::Node>(Eigen::Vector2d{0, 1}, Eigen::Vector2d{1, 2});
  auto ne = std::make_unique<bh::Node>(Eigen::Vector2d{1, 1}, Eigen::Vector2d{2, 2});
  auto se = std::make_unique<bh::Node>(Eigen::Vector2d{1, 0}, Eigen::Vector2d{2, 1});
  auto sw = std::make_unique<bh::Node>(Eigen::Vector2d{0, 0}, Eigen::Vector2d{1, 1});

  const auto b1 = bh::Body{{0.5, 1.5}, 0.25};
  nw->insert(b1);

  const auto b2 = bh::Body{{1.5, 1.5}, 0.25};
  ne->insert(b2);

  WHEN("The quadtree nodes are merged") {
    const auto merged = merge_quadtrees(std::move(nw), std::move(ne), std::move(se), std::move(sw));

    REQUIRE(merged->bbox().min() == Eigen::Vector2d{0, 0});
    REQUIRE(merged->bbox().max() == Eigen::Vector2d{2, 2});
    REQUIRE(merged->center_of_mass() == (b1.m_position + b2.m_position) / 2);
    REQUIRE(merged->total_mass() == b1.m_mass + b2.m_mass);
    REQUIRE(merged->n_nodes() == 5);

    const auto& merged_data = std::get<bh::Node::Fork>(merged->data());

    THEN("The NW quadtree node, which contained a body, still contains a body in the merged quadtree") {
      const auto& lv1_nw = *merged_data.m_children[bh::Node::NW];
      const auto& lv1_nw_data = std::get<bh::Node::Leaf>(lv1_nw.data());
      REQUIRE(lv1_nw_data.m_body);
      REQUIRE(lv1_nw_data.m_body->m_position == b1.m_position);
      REQUIRE(lv1_nw_data.m_body->m_mass == b1.m_mass);
    }

    THEN("The NE quadtree node, which contained a body, still contains a body in the merged quadtree") {
      const auto& lv1_ne = *merged_data.m_children[bh::Node::NE];
      const auto& lv1_ne_data = std::get<bh::Node::Leaf>(lv1_ne.data());
      REQUIRE(lv1_ne_data.m_body);
      REQUIRE(lv1_ne_data.m_body->m_position == b2.m_position);
      REQUIRE(lv1_ne_data.m_body->m_mass == b2.m_mass);
    }
  }
}

// https://www.desmos.com/calculator/ylev6z9mmb?lang=it
// also see reconstruct_8x8_quadtree_grid.png
TEST_CASE("Reconstruct 8x8 quadtree grid") {
  bh::QuadtreeGrid matrix(8);
  for (int i = 0; i < 8; i++) {
    matrix[i].resize(8);
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix[i][j] = std::make_unique<bh::Node>(Eigen::Vector2d(j, 7 - i), Eigen::Vector2d(j + 1, 8 - i));
    }
  }

  for (int i = 4; i < 8; i++) {
    for (int j = 4; j < 8; j++) {
      matrix[i][j] = std::make_unique<bh::Node>(Eigen::Vector2d(j, 7 - i), Eigen::Vector2d(j + 1, 8 - i));
    }
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 4; j < 6; j++) {
      matrix[i][j] = std::make_unique<bh::Node>(Eigen::Vector2d(j, 7 - i), Eigen::Vector2d(j + 1, 8 - i));
    }
  }

  for (int i = 2; i < 4; i++) {
    for (int j = 6; j < 8; j++) {
      matrix[i][j] = std::make_unique<bh::Node>(Eigen::Vector2d(j, 7 - i), Eigen::Vector2d(j + 1, 8 - i));
    }
  }

  for (int i = 4; i < 6; i++) {
    for (int j = 0; j < 2; j++) {
      matrix[i][j] = std::make_unique<bh::Node>(Eigen::Vector2d(j, 7 - i), Eigen::Vector2d(j + 1, 8 - i));
    }
  }

  for (int i = 6; i < 8; i++) {
    for (int j = 2; j < 4; j++) {
      matrix[i][j] = std::make_unique<bh::Node>(Eigen::Vector2d(j, 7 - i), Eigen::Vector2d(j + 1, 8 - i));
    }
  }

  matrix[0][6] = std::make_unique<bh::Node>(Eigen::Vector2d(6, 7), Eigen::Vector2d(7, 8));

  matrix[1][7] = std::make_unique<bh::Node>(Eigen::Vector2d(7, 6), Eigen::Vector2d(8, 7));

  matrix[2][4] = std::make_unique<bh::Node>(Eigen::Vector2d(4, 5), Eigen::Vector2d(5, 6));

  matrix[2][5] = std::make_unique<bh::Node>(Eigen::Vector2d(5, 5), Eigen::Vector2d(6, 6));

  matrix[3][5] = std::make_unique<bh::Node>(Eigen::Vector2d(5, 4), Eigen::Vector2d(6, 5));

  matrix[4][2] = std::make_unique<bh::Node>(Eigen::Vector2d(2, 3), Eigen::Vector2d(3, 4));

  matrix[5][2] = std::make_unique<bh::Node>(Eigen::Vector2d(2, 2), Eigen::Vector2d(3, 3));

  matrix[5][3] = std::make_unique<bh::Node>(Eigen::Vector2d(3, 2), Eigen::Vector2d(4, 3));

  matrix[6][0] = std::make_unique<bh::Node>(Eigen::Vector2d(0, 1), Eigen::Vector2d(1, 2));

  matrix[7][1] = std::make_unique<bh::Node>(Eigen::Vector2d(1, 0), Eigen::Vector2d(2, 1));

  matrix[0][7] = std::make_unique<bh::Node>(Eigen::Vector2d(7, 7), Eigen::Vector2d(8, 8));
  matrix[0][7]->insert({{7.5, 7.5}, 0.25});

  matrix[1][6] = std::make_unique<bh::Node>(Eigen::Vector2d(6, 6), Eigen::Vector2d(7, 7));
  matrix[1][6]->insert({{6.5, 6.5}, 0.25});

  matrix[3][4] = std::make_unique<bh::Node>(Eigen::Vector2d(4, 4), Eigen::Vector2d(5, 5));
  matrix[3][4]->insert({{4.5, 4.5}, 0.25});

  matrix[4][3] = std::make_unique<bh::Node>(Eigen::Vector2d(3, 3), Eigen::Vector2d(4, 4));
  matrix[4][3]->insert({{3.5, 3.5}, 0.25});

  matrix[6][1] = std::make_unique<bh::Node>(Eigen::Vector2d(1, 1), Eigen::Vector2d(2, 2));
  matrix[6][1]->insert({{1.5, 1.5}, 0.25});

  matrix[7][0] = std::make_unique<bh::Node>(Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  matrix[7][0]->insert({{0.5, 0.5}, 0.25});

  const auto lv0 = bh::reconstruct_quadtree(matrix);
  REQUIRE(lv0->bbox().min() == Eigen::Vector2d{0, 0});
  REQUIRE(lv0->bbox().max() == Eigen::Vector2d{8, 8});
  REQUIRE(lv0->center_of_mass() == Eigen::Vector2d{4, 4});
  REQUIRE(lv0->total_mass() == 1.5);
  REQUIRE(lv0->n_nodes() == 21);
  const auto& lv0_data = std::get<bh::Node::Fork>(lv0->data());

  SECTION("lv1-NW") {
    const auto& lv1_nw = *lv0_data.m_children[bh::Node::NW];
    REQUIRE(lv1_nw.bbox().min() == Eigen::Vector2d{0, 4});
    REQUIRE(lv1_nw.bbox().max() == Eigen::Vector2d{4, 8});
    REQUIRE(lv1_nw.n_nodes() == 1);
    REQUIRE(lv1_nw.center_of_mass() == Eigen::Vector2d{0, 0});
    REQUIRE(lv1_nw.total_mass() == 0);
  }

  SECTION("lv1-NE") {
    const auto& lv1_ne = *lv0_data.m_children[bh::Node::NE];
    REQUIRE(lv1_ne.bbox().min() == Eigen::Vector2d{4, 4});
    REQUIRE(lv1_ne.bbox().max() == Eigen::Vector2d{8, 8});
    REQUIRE(lv1_ne.n_nodes() == 9);
    REQUIRE(lv1_ne.center_of_mass().x() == Catch::Approx(6.16666666667));
    REQUIRE(lv1_ne.center_of_mass().y() == Catch::Approx(6.16666666667));
    REQUIRE(lv1_ne.total_mass() == 0.25 * 3);
    const auto& lv1_ne_data = std::get<bh::Node::Fork>(lv1_ne.data());

    SECTION("lv2-NW") {
      const auto& lv2_nw = *lv1_ne_data.m_children[bh::Node::NW];
      REQUIRE(lv2_nw.bbox().min() == Eigen::Vector2d{4, 6});
      REQUIRE(lv2_nw.bbox().max() == Eigen::Vector2d{6, 8});
      REQUIRE(lv2_nw.n_nodes() == 1);
      REQUIRE(lv2_nw.center_of_mass() == Eigen::Vector2d{0, 0});
      REQUIRE(lv2_nw.total_mass() == 0);
    }

    SECTION("lv2-NE") {
      const auto& lv2_ne = *lv1_ne_data.m_children[bh::Node::NE];
      REQUIRE(lv2_ne.bbox().min() == Eigen::Vector2d{6, 6});
      REQUIRE(lv2_ne.bbox().max() == Eigen::Vector2d{8, 8});
      REQUIRE(lv2_ne.n_nodes() == 5);
      REQUIRE(lv2_ne.center_of_mass() == Eigen::Vector2d{7, 7});
      REQUIRE(lv2_ne.total_mass() == 2 * 0.25);
      const auto& lv2_ne_data = std::get<bh::Node::Fork>(lv2_ne.data());

      SECTION("lv3-NW") {
        const auto& lv3_nw = *lv2_ne_data.m_children[bh::Node::NW];
        REQUIRE(lv3_nw.bbox().min() == Eigen::Vector2d{6, 7});
        REQUIRE(lv3_nw.bbox().max() == Eigen::Vector2d{7, 8});
        REQUIRE(lv3_nw.n_nodes() == 1);
        REQUIRE(lv3_nw.center_of_mass() == Eigen::Vector2d{0, 0});
        REQUIRE(lv3_nw.total_mass() == 0);
      }

      SECTION("lv3-NE") {  // contains the black node in the Desmos graph
        const auto& lv3_ne = *lv2_ne_data.m_children[bh::Node::NE];
        REQUIRE(lv3_ne.bbox().min() == Eigen::Vector2d{7, 7});
        REQUIRE(lv3_ne.bbox().max() == Eigen::Vector2d{8, 8});
        REQUIRE(lv3_ne.n_nodes() == 1);
        REQUIRE(lv3_ne.center_of_mass() == Eigen::Vector2d{7.5, 7.5});
        REQUIRE(lv3_ne.total_mass() == 0.25);
        const auto& lv3_ne_data = std::get<bh::Node::Leaf>(lv3_ne.data());
        REQUIRE(lv3_ne_data.m_body);
      }

      SECTION("lv3-SE") {
        const auto& lv3_se = *lv2_ne_data.m_children[bh::Node::SE];
        REQUIRE(lv3_se.bbox().min() == Eigen::Vector2d{7, 6});
        REQUIRE(lv3_se.bbox().max() == Eigen::Vector2d{8, 7});
        REQUIRE(lv3_se.n_nodes() == 1);
        REQUIRE(lv3_se.center_of_mass() == Eigen::Vector2d{0, 0});
        REQUIRE(lv3_se.total_mass() == 0);
      }

      SECTION("lv3-SW") {  // contains the purple node in the Desmos graph
        const auto& lv3_sw = *lv2_ne_data.m_children[bh::Node::SW];
        REQUIRE(lv3_sw.bbox().min() == Eigen::Vector2d{6, 6});
        REQUIRE(lv3_sw.bbox().max() == Eigen::Vector2d{7, 7});
        REQUIRE(lv3_sw.n_nodes() == 1);
        REQUIRE(lv3_sw.center_of_mass() == Eigen::Vector2d{6.5, 6.5});
        REQUIRE(lv3_sw.total_mass() == 0.25);
        const auto& lv3_sw_data = std::get<bh::Node::Leaf>(lv3_sw.data());
        REQUIRE(lv3_sw_data.m_body);
      }
    }

    SECTION("lv2-SE") {
      const auto& lv2_se = *lv1_ne_data.m_children[bh::Node::SE];
      REQUIRE(lv2_se.bbox().min() == Eigen::Vector2d{6, 4});
      REQUIRE(lv2_se.bbox().max() == Eigen::Vector2d{8, 6});
      REQUIRE(lv2_se.n_nodes() == 1);
      REQUIRE(lv2_se.center_of_mass() == Eigen::Vector2d{0, 0});
      REQUIRE(lv2_se.total_mass() == 0);
    }

    SECTION("lv2-SW") {  // contains the orange node in the Desmos graph
      const auto& lv2_sw = *lv1_ne_data.m_children[bh::Node::SW];
      REQUIRE(lv2_sw.bbox().min() == Eigen::Vector2d{4, 4});
      REQUIRE(lv2_sw.bbox().max() == Eigen::Vector2d{6, 6});
      REQUIRE(lv2_sw.n_nodes() == 1);
      REQUIRE(lv2_sw.center_of_mass() == Eigen::Vector2d{4.5, 4.5});
      REQUIRE(lv2_sw.total_mass() == 0.25);
      const auto& lv2_sw_data = std::get<bh::Node::Leaf>(lv2_sw.data());
      REQUIRE(lv2_sw_data.m_body);
    }
  }

  SECTION("lv1-SE") {
    const auto& lv1_se = *lv0_data.m_children[bh::Node::SE];
    REQUIRE(lv1_se.bbox().min() == Eigen::Vector2d{4, 0});
    REQUIRE(lv1_se.bbox().max() == Eigen::Vector2d{8, 4});
    REQUIRE(lv1_se.n_nodes() == 1);
    REQUIRE(lv1_se.center_of_mass() == Eigen::Vector2d{0, 0});
    REQUIRE(lv1_se.total_mass() == 0);
  }

  SECTION("lv1-SW") {
    const auto& lv1_sw = *lv0_data.m_children[bh::Node::SW];
    REQUIRE(lv1_sw.bbox().min() == Eigen::Vector2d{0, 0});
    REQUIRE(lv1_sw.bbox().max() == Eigen::Vector2d{4, 4});
    REQUIRE(lv1_sw.n_nodes() == 9);
    REQUIRE(lv1_sw.center_of_mass().x() == Catch::Approx(1.83333333333));
    REQUIRE(lv1_sw.center_of_mass().y() == Catch::Approx(1.83333333333));
    REQUIRE(lv1_sw.total_mass() == 0.25 * 3);
    const auto& lv1_sw_data = std::get<bh::Node::Fork>(lv1_sw.data());

    SECTION("lv2-NW") {
      const auto& lv2_nw = *lv1_sw_data.m_children[bh::Node::NW];
      REQUIRE(lv2_nw.bbox().min() == Eigen::Vector2d{0, 2});
      REQUIRE(lv2_nw.bbox().max() == Eigen::Vector2d{2, 4});
      REQUIRE(lv2_nw.n_nodes() == 1);
      REQUIRE(lv2_nw.center_of_mass() == Eigen::Vector2d{0, 0});
      REQUIRE(lv2_nw.total_mass() == 0);
    }

    SECTION("lv2-NE") {  // contains the green node in the Desmos graph
      const auto& lv2_ne = *lv1_sw_data.m_children[bh::Node::NE];
      REQUIRE(lv2_ne.bbox().min() == Eigen::Vector2d{2, 2});
      REQUIRE(lv2_ne.bbox().max() == Eigen::Vector2d{4, 4});
      REQUIRE(lv2_ne.n_nodes() == 1);
      REQUIRE(lv2_ne.center_of_mass() == Eigen::Vector2d{3.5, 3.5});
      REQUIRE(lv2_ne.total_mass() == 0.25);
      const auto& lv2_ne_data = std::get<bh::Node::Leaf>(lv2_ne.data());
      REQUIRE(lv2_ne_data.m_body);
    }

    SECTION("lv2-SE") {
      const auto& lv2_se = *lv1_sw_data.m_children[bh::Node::SE];
      REQUIRE(lv2_se.bbox().min() == Eigen::Vector2d{2, 0});
      REQUIRE(lv2_se.bbox().max() == Eigen::Vector2d{4, 2});
      REQUIRE(lv2_se.n_nodes() == 1);
      REQUIRE(lv2_se.center_of_mass() == Eigen::Vector2d{0, 0});
      REQUIRE(lv2_se.total_mass() == 0);
    }

    SECTION("lv2-SW") {
      const auto& lv2_sw = *lv1_sw_data.m_children[bh::Node::SW];
      REQUIRE(lv2_sw.bbox().min() == Eigen::Vector2d{0, 0});
      REQUIRE(lv2_sw.bbox().max() == Eigen::Vector2d{2, 2});
      REQUIRE(lv2_sw.n_nodes() == 5);
      REQUIRE(lv2_sw.center_of_mass() == Eigen::Vector2d{1, 1});
      REQUIRE(lv2_sw.total_mass() == 2 * 0.25);
      const auto& lv2_sw_data = std::get<bh::Node::Fork>(lv2_sw.data());

      SECTION("lv3-NW") {
        const auto& lv3_nw = *lv2_sw_data.m_children[bh::Node::NW];
        REQUIRE(lv3_nw.bbox().min() == Eigen::Vector2d{0, 1});
        REQUIRE(lv3_nw.bbox().max() == Eigen::Vector2d{1, 2});
        REQUIRE(lv3_nw.n_nodes() == 1);
        REQUIRE(lv3_nw.center_of_mass() == Eigen::Vector2d{0, 0});
        REQUIRE(lv3_nw.total_mass() == 0);
      }

      SECTION("lv3-NE") {  // contains the blue node in the Desmos graph
        const auto& lv3_ne = *lv2_sw_data.m_children[bh::Node::NE];
        REQUIRE(lv3_ne.bbox().min() == Eigen::Vector2d{1, 1});
        REQUIRE(lv3_ne.bbox().max() == Eigen::Vector2d{2, 2});
        REQUIRE(lv3_ne.n_nodes() == 1);
        REQUIRE(lv3_ne.center_of_mass() == Eigen::Vector2d{1.5, 1.5});
        REQUIRE(lv3_ne.total_mass() == 0.25);
        const auto& lv3_ne_data = std::get<bh::Node::Leaf>(lv3_ne.data());
        REQUIRE(lv3_ne_data.m_body);
      }

      SECTION("lv3-SE") {
        const auto& lv3_se = *lv2_sw_data.m_children[bh::Node::SE];
        REQUIRE(lv3_se.bbox().min() == Eigen::Vector2d{1, 0});
        REQUIRE(lv3_se.bbox().max() == Eigen::Vector2d{2, 1});
        REQUIRE(lv3_se.n_nodes() == 1);
        REQUIRE(lv3_se.center_of_mass() == Eigen::Vector2d{0, 0});
        REQUIRE(lv3_se.total_mass() == 0);
      }

      SECTION("lv3-SW") {  // contains the red node in the Desmos graph
        const auto& lv3_sw = *lv2_sw_data.m_children[bh::Node::SW];
        REQUIRE(lv3_sw.bbox().min() == Eigen::Vector2d{0, 0});
        REQUIRE(lv3_sw.bbox().max() == Eigen::Vector2d{1, 1});
        REQUIRE(lv3_sw.n_nodes() == 1);
        REQUIRE(lv3_sw.center_of_mass() == Eigen::Vector2d{0.5, 0.5});
        REQUIRE(lv3_sw.total_mass() == 0.25);
        const auto& lv3_sw_data = std::get<bh::Node::Leaf>(lv3_sw.data());
        REQUIRE(lv3_sw_data.m_body);
      }
    }
  }
}
