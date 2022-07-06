#include <catch2/catch_test_macros.hpp>

#include "mpi_barnes_hut_simulator.h"

TEST_CASE("compute bounding box for 16 processors; non-negative coordinates") {
  const Eigen::AlignedBox2d outer{Eigen::Vector2d{0, 0}, Eigen::Vector2d{10, 10}};

  auto q0 = bh::compute_bounding_box_for_processor(outer, 0, 16);
  auto q1 = bh::compute_bounding_box_for_processor(outer, 1, 16);
  auto q2 = bh::compute_bounding_box_for_processor(outer, 2, 16);
  auto q3 = bh::compute_bounding_box_for_processor(outer, 3, 16);
  auto q4 = bh::compute_bounding_box_for_processor(outer, 4, 16);
  auto q5 = bh::compute_bounding_box_for_processor(outer, 5, 16);
  auto q6 = bh::compute_bounding_box_for_processor(outer, 6, 16);
  auto q7 = bh::compute_bounding_box_for_processor(outer, 7, 16);
  auto q8 = bh::compute_bounding_box_for_processor(outer, 8, 16);
  auto q9 = bh::compute_bounding_box_for_processor(outer, 9, 16);
  auto q10 = bh::compute_bounding_box_for_processor(outer, 10, 16);
  auto q11 = bh::compute_bounding_box_for_processor(outer, 11, 16);
  auto q12 = bh::compute_bounding_box_for_processor(outer, 12, 16);
  auto q13 = bh::compute_bounding_box_for_processor(outer, 13, 16);
  auto q14 = bh::compute_bounding_box_for_processor(outer, 14, 16);
  auto q15 = bh::compute_bounding_box_for_processor(outer, 15, 16);

  REQUIRE(q0.min() == Eigen::Vector2d{0, 0});
  REQUIRE(q0.max() == Eigen::Vector2d{2.5, 2.5});

  REQUIRE(q1.min() == Eigen::Vector2d{2.5, 0});
  REQUIRE(q1.max() == Eigen::Vector2d{5, 2.5});

  REQUIRE(q2.min() == Eigen::Vector2d{5, 0});
  REQUIRE(q2.max() == Eigen::Vector2d{7.5, 2.5});

  REQUIRE(q3.min() == Eigen::Vector2d{7.5, 0});
  REQUIRE(q3.max() == Eigen::Vector2d{10, 2.5});

  REQUIRE(q4.min() == Eigen::Vector2d{0, 2.5});
  REQUIRE(q4.max() == Eigen::Vector2d{2.5, 5});

  REQUIRE(q5.min() == Eigen::Vector2d{2.5, 2.5});
  REQUIRE(q5.max() == Eigen::Vector2d{5, 5});

  REQUIRE(q6.min() == Eigen::Vector2d{5, 2.5});
  REQUIRE(q6.max() == Eigen::Vector2d{7.5, 5});

  REQUIRE(q7.min() == Eigen::Vector2d{7.5, 2.5});
  REQUIRE(q7.max() == Eigen::Vector2d{10, 5});

  REQUIRE(q8.min() == Eigen::Vector2d{0, 5});
  REQUIRE(q8.max() == Eigen::Vector2d{2.5, 7.5});

  REQUIRE(q9.min() == Eigen::Vector2d{2.5, 5});
  REQUIRE(q9.max() == Eigen::Vector2d{5, 7.5});

  REQUIRE(q10.min() == Eigen::Vector2d{5, 5});
  REQUIRE(q10.max() == Eigen::Vector2d{7.5, 7.5});

  REQUIRE(q11.min() == Eigen::Vector2d{7.5, 5});
  REQUIRE(q11.max() == Eigen::Vector2d{10, 7.5});

  REQUIRE(q12.min() == Eigen::Vector2d{0, 7.5});
  REQUIRE(q12.max() == Eigen::Vector2d{2.5, 10});

  REQUIRE(q13.min() == Eigen::Vector2d{2.5, 7.5});
  REQUIRE(q13.max() == Eigen::Vector2d{5, 10});

  REQUIRE(q14.min() == Eigen::Vector2d{5, 7.5});
  REQUIRE(q14.max() == Eigen::Vector2d{7.5, 10});

  REQUIRE(q15.min() == Eigen::Vector2d{7.5, 7.5});
  REQUIRE(q15.max() == Eigen::Vector2d{10, 10});
}

TEST_CASE("compute bounding box for 4 processors; positive and negative coordinates") {
  const Eigen::AlignedBox2d outer{Eigen::Vector2d{-2, -2}, Eigen::Vector2d{2, 2}};

  auto q0 = bh::compute_bounding_box_for_processor(outer, 0, 4);
  auto q1 = bh::compute_bounding_box_for_processor(outer, 1, 4);
  auto q2 = bh::compute_bounding_box_for_processor(outer, 2, 4);
  auto q3 = bh::compute_bounding_box_for_processor(outer, 3, 4);

  REQUIRE(q0.min() == Eigen::Vector2d{-2, -2});
  REQUIRE(q0.max() == Eigen::Vector2d{0, 0});

  REQUIRE(q1.min() == Eigen::Vector2d{0, -2});
  REQUIRE(q1.max() == Eigen::Vector2d{2, 0});

  REQUIRE(q2.min() == Eigen::Vector2d{-2, 0});
  REQUIRE(q2.max() == Eigen::Vector2d{0, 2});

  REQUIRE(q3.min() == Eigen::Vector2d{0, 0});
  REQUIRE(q3.max() == Eigen::Vector2d{2, 2});
}

TEST_CASE("compute bounding box for 4 processors; negative coordinates only") {
  const Eigen::AlignedBox2d outer{Eigen::Vector2d{-4, -4}, Eigen::Vector2d{-2, -2}};

  auto q0 = bh::compute_bounding_box_for_processor(outer, 0, 4);
  auto q1 = bh::compute_bounding_box_for_processor(outer, 1, 4);
  auto q2 = bh::compute_bounding_box_for_processor(outer, 2, 4);
  auto q3 = bh::compute_bounding_box_for_processor(outer, 3, 4);

  REQUIRE(q0.min() == Eigen::Vector2d{-4, -4});
  REQUIRE(q0.max() == Eigen::Vector2d{-3, -3});

  REQUIRE(q1.min() == Eigen::Vector2d{-3, -4});
  REQUIRE(q1.max() == Eigen::Vector2d{-2, -3});

  REQUIRE(q2.min() == Eigen::Vector2d{-4, -3});
  REQUIRE(q2.max() == Eigen::Vector2d{-3, -2});

  REQUIRE(q3.min() == Eigen::Vector2d{-3, -3});
  REQUIRE(q3.max() == Eigen::Vector2d{-2, -2});
}

