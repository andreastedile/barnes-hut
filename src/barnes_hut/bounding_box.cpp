#include "bounding_box.h"

#include <eigen3/Eigen/Eigen>
#include <execution>  // par_unseq
#include <numeric>    // transform_reduce

using Eigen::Vector2d;

namespace bh {

AlignedBox2d compute_minimum_bounding_box(const std::vector<Body> &bodies) {
  if (bodies.size() < 2) {
    throw std::invalid_argument(
        "cannot compute the minimum bounding box for less than two bodies");
  }

  // Compute the rectangular bounding box containing all bodies
  Vector2d bottom_left = std::transform_reduce(
      std::execution::par_unseq, bodies.begin(), bodies.end(),
      // Vector2d(Eigen::Infinity, Eigen::Infinity), // does not work
      Vector2d(std::numeric_limits<double>::max(),
               std::numeric_limits<double>::max()),
      [](const Vector2d &bottom_left, const Vector2d &body) {
        return Vector2d(std::min(bottom_left.x(), body.x()),
                        std::min(bottom_left.y(), body.y()));
      },
      [](const Body &body) { return body.m_position; });
  Vector2d top_right = std::transform_reduce(
      std::execution::par_unseq, bodies.begin(), bodies.end(),
      // // Vector2d(Eigen::Infinity, Eigen::Infinity), // does not work
      Vector2d(std::numeric_limits<double>::min(),
               std::numeric_limits<double>::min()),
      [](const Vector2d &top_right, const Vector2d &body) {
        return Vector2d(std::max(top_right.x(), body.x()),
                        std::max(top_right.y(), body.y()));
      },
      [](const Body &body) { return body.m_position; });

  return {bottom_left, top_right};
}

AlignedBox2d compute_square_bounding_box(const std::vector<Body> &bodies) {
  AlignedBox2d min_bbox = compute_minimum_bounding_box(bodies);
  Vector2d bottom_left = min_bbox.min();
  Vector2d top_right = min_bbox.max();
  Vector2d top_left(bottom_left.x(), top_right.y());
  Vector2d bottom_right(top_right.x(), bottom_left.y());

  // Usually, the minimum bounding box is a rectangle. In this case, we have to
  // convert it into a square.
  double width = (top_right - top_left).norm();
  double height = (top_left - bottom_left).norm();
  if (width > height) {
    double diff = width - height;
    bottom_left -= Vector2d(0, diff / 2);
    bottom_right -= Vector2d(0, diff / 2);
    top_right += Vector2d(0, diff / 2);
    top_left += Vector2d(0, diff / 2);
  } else if (height > width) {
    double diff = height - width;
    bottom_left -= Vector2d(diff / 2, 0);
    top_left -= Vector2d(diff / 2, 0);
    bottom_right += Vector2d(diff / 2, 0);
    top_right += Vector2d(diff / 2, 0);
  }

  // We now snap the square's corners to the grid, but this may produce a
  // rectangle.
  bottom_left = {std::floor(bottom_left.x()), std::floor(bottom_left.y())};
  top_right = {std::ceil(top_right.x()), std::ceil(top_right.y())};
  top_left = {std::floor(top_left.x()), std::ceil(top_left.y())};
  bottom_right = {std::ceil(bottom_right.x()), std::floor(bottom_right.y())};

  // We now extend the rectangle into a square.
  width = (top_right - top_left).norm();
  height = (top_left - bottom_left).norm();
  if (width > height) {
    double diff = width - height;
    top_right += Vector2d(0, diff);
    top_left += Vector2d(0, diff);
  } else if (height > width) {
    double diff = height - width;
    top_right += Vector2d(diff, 0);
    bottom_right += Vector2d(diff, 0);
  }

  return {bottom_left, top_right};
}

}  // namespace bh
