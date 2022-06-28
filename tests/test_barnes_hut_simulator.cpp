#include <catch2/catch.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include "mpi_barnes_hut_simulator.h"

using Eigen::AlignedBox2d;
using Eigen::Vector2d;

TEST_CASE("compute bounding box for 16 processors; non-negative coordinates") {
  const AlignedBox2d outer{Vector2d{0, 0}, Vector2d{10, 10}};

  auto q0 = bh::compute_bounding_box_for_processor(outer, 16, 0);
  auto q1 = bh::compute_bounding_box_for_processor(outer, 16, 1);
  auto q2 = bh::compute_bounding_box_for_processor(outer, 16, 2);
  auto q3 = bh::compute_bounding_box_for_processor(outer, 16, 3);
  auto q4 = bh::compute_bounding_box_for_processor(outer, 16, 4);
  auto q5 = bh::compute_bounding_box_for_processor(outer, 16, 5);
  auto q6 = bh::compute_bounding_box_for_processor(outer, 16, 6);
  auto q7 = bh::compute_bounding_box_for_processor(outer, 16, 7);
  auto q8 = bh::compute_bounding_box_for_processor(outer, 16, 8);
  auto q9 = bh::compute_bounding_box_for_processor(outer, 16, 9);
  auto q10 = bh::compute_bounding_box_for_processor(outer, 16, 10);
  auto q11 = bh::compute_bounding_box_for_processor(outer, 16, 11);
  auto q12 = bh::compute_bounding_box_for_processor(outer, 16, 12);
  auto q13 = bh::compute_bounding_box_for_processor(outer, 16, 13);
  auto q14 = bh::compute_bounding_box_for_processor(outer, 16, 14);
  auto q15 = bh::compute_bounding_box_for_processor(outer, 16, 15);

  REQUIRE(q0.min() == Vector2d{0, 0});
  REQUIRE(q0.max() == Vector2d{2.5, 2.5});

  REQUIRE(q1.min() == Vector2d{2.5, 0});
  REQUIRE(q1.max() == Vector2d{5, 2.5});

  REQUIRE(q2.min() == Vector2d{5, 0});
  REQUIRE(q2.max() == Vector2d{7.5, 2.5});

  REQUIRE(q3.min() == Vector2d{7.5, 0});
  REQUIRE(q3.max() == Vector2d{10, 2.5});

  REQUIRE(q4.min() == Vector2d{0, 2.5});
  REQUIRE(q4.max() == Vector2d{2.5, 5});

  REQUIRE(q5.min() == Vector2d{2.5, 2.5});
  REQUIRE(q5.max() == Vector2d{5, 5});

  REQUIRE(q6.min() == Vector2d{5, 2.5});
  REQUIRE(q6.max() == Vector2d{7.5, 5});

  REQUIRE(q7.min() == Vector2d{7.5, 2.5});
  REQUIRE(q7.max() == Vector2d{10, 5});

  REQUIRE(q8.min() == Vector2d{0, 5});
  REQUIRE(q8.max() == Vector2d{2.5, 7.5});

  REQUIRE(q9.min() == Vector2d{2.5, 5});
  REQUIRE(q9.max() == Vector2d{5, 7.5});

  REQUIRE(q10.min() == Vector2d{5, 5});
  REQUIRE(q10.max() == Vector2d{7.5, 7.5});

  REQUIRE(q11.min() == Vector2d{7.5, 5});
  REQUIRE(q11.max() == Vector2d{10, 7.5});

  REQUIRE(q12.min() == Vector2d{0, 7.5});
  REQUIRE(q12.max() == Vector2d{2.5, 10});

  REQUIRE(q13.min() == Vector2d{2.5, 7.5});
  REQUIRE(q13.max() == Vector2d{5, 10});

  REQUIRE(q14.min() == Vector2d{5, 7.5});
  REQUIRE(q14.max() == Vector2d{7.5, 10});

  REQUIRE(q15.min() == Vector2d{7.5, 7.5});
  REQUIRE(q15.max() == Vector2d{10, 10});
}

TEST_CASE("compute bounding box for 4 processors; positive and negative coordinates") {
  const AlignedBox2d outer{Vector2d{-2, -2}, Vector2d{2, 2}};

  auto q0 = bh::compute_bounding_box_for_processor(outer, 4, 0);
  auto q1 = bh::compute_bounding_box_for_processor(outer, 4, 1);
  auto q2 = bh::compute_bounding_box_for_processor(outer, 4, 2);
  auto q3 = bh::compute_bounding_box_for_processor(outer, 4, 3);

  REQUIRE(q0.min() == Vector2d{-2, -2});
  REQUIRE(q0.max() == Vector2d{0, 0});

  REQUIRE(q1.min() == Vector2d{0, -2});
  REQUIRE(q1.max() == Vector2d{2, 0});

  REQUIRE(q2.min() == Vector2d{-2, 0});
  REQUIRE(q2.max() == Vector2d{0, 2});

  REQUIRE(q3.min() == Vector2d{0, 0});
  REQUIRE(q3.max() == Vector2d{2, 2});
}

TEST_CASE("compute bounding box for 4 processors; negative coordinates only") {
  const AlignedBox2d outer{Vector2d{-4, -4}, Vector2d{-2, -2}};

  auto q0 = bh::compute_bounding_box_for_processor(outer, 4, 0);
  auto q1 = bh::compute_bounding_box_for_processor(outer, 4, 1);
  auto q2 = bh::compute_bounding_box_for_processor(outer, 4, 2);
  auto q3 = bh::compute_bounding_box_for_processor(outer, 4, 3);

  REQUIRE(q0.min() == Vector2d{-4, -4});
  REQUIRE(q0.max() == Vector2d{-3, -3});

  REQUIRE(q1.min() == Vector2d{-3, -4});
  REQUIRE(q1.max() == Vector2d{-2, -3});

  REQUIRE(q2.min() == Vector2d{-4, -3});
  REQUIRE(q2.max() == Vector2d{-3, -2});

  REQUIRE(q3.min() == Vector2d{-3, -3});
  REQUIRE(q3.max() == Vector2d{-2, -2});
}
