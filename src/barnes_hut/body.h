#ifndef BARNES_HUT_BODY_H
#define BARNES_HUT_BODY_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <execution>
#include <numeric>      // reduce
#include <type_traits>  // enable_if, is_base_of

using Eigen::AlignedBox2d;
using Eigen::Vector2d;

namespace bh {

struct Body {
  Vector2d m_position;
  double m_mass;
  Body(Vector2d position, double mass);
  Body();
};

// We implement the function in the header itself.
// https://stackoverflow.com/questions/10632251/undefined-reference-to-template-function
template <typename T, typename = typename std::enable_if<
                          std::is_base_of<Body, T>::value, T>::type>
/**
 * Computes the minimum bounding box (usually, a rectangle) containing the
 * bodies.
 */
AlignedBox2d compute_minimum_bounding_box(const std::vector<T> &bodies) {
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
      [](const T &body) { return body.m_position; });
  Vector2d top_right = std::transform_reduce(
      std::execution::par_unseq, bodies.begin(), bodies.end(),
      // // Vector2d(Eigen::Infinity, Eigen::Infinity), // does not work
      Vector2d(std::numeric_limits<double>::min(),
               std::numeric_limits<double>::min()),
      [](const Vector2d &top_right, const Vector2d &body) {
        return Vector2d(std::max(top_right.x(), body.x()),
                        std::max(top_right.y(), body.y()));
      },
      [](const T &body) { return body.m_position; });

  return {bottom_left, top_right};
}

template <typename T, typename = typename std::enable_if<
                          std::is_base_of<Body, T>::value, T>::type>
AlignedBox2d compute_square_bounding_box(const std::vector<T> &bodies) {
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

#endif  // BARNES_HUT_BODY_H
