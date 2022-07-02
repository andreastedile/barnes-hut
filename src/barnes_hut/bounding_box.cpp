#include "bounding_box.h"

#include <eigen3/Eigen/Eigen>
#include <execution>  // par_unseq
#include <numeric>    // transform_reduce

using Eigen::Vector2d;

namespace bh {

AlignedBox2d compute_minimum_bounding_box(const std::vector<Body> &bodies) {
  if (bodies.empty()) {
    return {Vector2d{0, 0}, Vector2d{0, 0}};
  } else if (bodies.size() == 1) {
    return {bodies[0].m_position, bodies[0].m_position};
  } else {
    // Compute the (typically rectangular) bounding box containing all bodies
    auto bottom_left = std::transform_reduce(
        std::execution::par_unseq,
        bodies.begin(), bodies.end(),
        Vector2d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
        [](const Vector2d &bottom_left, const Vector2d &body) -> Vector2d {
          return {std::min(bottom_left.x(), body.x()), std::min(bottom_left.y(), body.y())};
        },
        [](const Body &body) {
          return body.m_position;
        });
    auto top_right = std::transform_reduce(
        std::execution::par_unseq,
        bodies.begin(), bodies.end(),
        Vector2d(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()),
        [](const Vector2d &top_right, const Vector2d &body) -> Vector2d {
          return {std::max(top_right.x(), body.x()), std::max(top_right.y(), body.y())};
        },
        [](const Body &body) {
          return body.m_position;
        });

    return {bottom_left, top_right};
  }
}

AlignedBox2d compute_square_bounding_box(const std::vector<Body> &bodies) {
  AlignedBox2d min_bbox = compute_minimum_bounding_box(bodies);

  if (double diff = min_bbox.sizes().x() > min_bbox.sizes().y(); diff > 0) {
    // width > height
    min_bbox.min() -= Vector2d(0, diff / 2);
    min_bbox.max() += Vector2d(0, diff / 2);
  } else if ( diff = min_bbox.sizes().y() > min_bbox.sizes().x(); diff > 0) {
    // height > width
    min_bbox.min() -= Vector2d(diff / 2, 0);
    min_bbox.max() += Vector2d(diff / 2, 0);
  } else if (min_bbox.min() == min_bbox.max()) {
    // bounding box degenerated to a point
    min_bbox.min() += Vector2d(-0.5, -0.5);
    min_bbox.max() += Vector2d(0.5, 0.5);
  }

  // We now snap the square's corners to the grid, but this may produce a rectangle.
  min_bbox.min() = {std::floor(min_bbox.min().x()), std::floor(min_bbox.min().y())};
  min_bbox.max() = {std::ceil(min_bbox.max().x()), std::ceil(min_bbox.max().y())};

  // We now extend the rectangle into a square.
  if (double diff = min_bbox.sizes().x() - min_bbox.sizes().y(); diff > 0) {
    min_bbox.max() += Vector2d(0, diff);
  } else if (diff = min_bbox.sizes().y() - min_bbox.sizes().x(); diff > 0) {
    min_bbox.max() += Vector2d(diff, 0);
  }

  return min_bbox;
}

}  // namespace bh
