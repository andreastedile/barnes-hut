#include <catch2/catch_test_macros.hpp>
#include <Eigen/Geometry>
#include <limits>

SCENARIO("eigen aligned box") {
  GIVEN("A bounding box") {
    const Eigen::AlignedBox2d box(Eigen::Vector2d{0, 0}, Eigen::Vector2d{10, 10});

    THEN("The minimum is") {
      REQUIRE(box.min() == Eigen::Vector2d{0, 0});
      REQUIRE(box.max() == Eigen::Vector2d{10, 10});
    }

    THEN("The center is") {
      REQUIRE(box.center() == Eigen::Vector2d{5, 5});
    }

    THEN("The corners are") {
      REQUIRE(box.corner(box.TopLeft) == Eigen::Vector2d{0, 10});
      REQUIRE(box.corner(box.TopRight) == Eigen::Vector2d{10, 10});
      REQUIRE(box.corner(box.BottomRight) == Eigen::Vector2d{10, 0});
      REQUIRE(box.corner(box.BottomLeft) == Eigen::Vector2d{0, 0});
    }

    THEN("The sides are") {
      REQUIRE(box.sizes().x() == 10);
      REQUIRE(box.sizes().y() == 10);
    }

    THEN("It contains any point which lies on its edges") {
      REQUIRE(box.contains(Eigen::Vector2d{0, 5}));
      REQUIRE(box.contains(Eigen::Vector2d{5, 10}));
      REQUIRE(box.contains(Eigen::Vector2d{10, 5}));
      REQUIRE(box.contains(Eigen::Vector2d{5, 0}));
    }

    THEN("It contains any point which lies on its corners") {
      REQUIRE(box.contains(Eigen::Vector2d{0, 0}));
      REQUIRE(box.contains(Eigen::Vector2d{0, 10}));
      REQUIRE(box.contains(Eigen::Vector2d{10, 10}));
      REQUIRE(box.contains(Eigen::Vector2d{10, 0}));
    }

    THEN("It does not contain points slightly outside of its edges") {
      constexpr double DBL_MIN = std::numeric_limits<double>::max();
      // Points slightly outside of the bounding box
      REQUIRE_FALSE(box.contains(Eigen::Vector2d{0, 5 - DBL_MIN}));
      REQUIRE_FALSE(box.contains(Eigen::Vector2d{5, 10 + DBL_MIN}));
      REQUIRE_FALSE(box.contains(Eigen::Vector2d{10 + DBL_MIN, 5}));
      REQUIRE_FALSE(box.contains(Eigen::Vector2d{5, 0 - DBL_MIN}));
    }
  }
}
