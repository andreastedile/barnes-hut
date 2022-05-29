#ifndef BARNES_HUT_BODY_H
#define BARNES_HUT_BODY_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <numeric>      // accumulate
#include <type_traits>  // enable_if, is_base_of

using Eigen::AlignedBox2f;
using Eigen::Vector2f;

namespace bh {

struct Body {
  const Vector2f m_position;
  const float m_mass;
  Body(Vector2f position, float mass);
};

// We implement the function in the header itself.
// https://stackoverflow.com/questions/10632251/undefined-reference-to-template-function
template <typename T, typename = typename std::enable_if<
                          std::is_base_of<Body, T>::value, T>::type>
/**
 * Computes the minimum bounding box (usually, a rectangle) containing the
 * bodies.
 */
AlignedBox2f compute_minimum_bounding_box(const std::vector<T> &bodies) {
  if (bodies.size() < 2) {
    throw std::invalid_argument(
        "cannot compute the minimum bounding box for less than two bodies");
  }

  // Compute the rectangular bounding box containing all bodies
  Vector2f bottom_left = std::accumulate(
      bodies.begin(), bodies.end(),
      // Vector2f(Eigen::Infinity, Eigen::Infinity), // does not work
      Vector2f(std::numeric_limits<float>::max(),
               std::numeric_limits<float>::max()),
      [](const Vector2f &bottom_left, const T &body) {
        return Vector2f(std::min(bottom_left.x(), body.m_position.x()),
                        std::min(bottom_left.y(), body.m_position.y()));
      });
  Vector2f top_right = std::accumulate(
      bodies.begin(), bodies.end(),
      // // Vector2f(Eigen::Infinity, Eigen::Infinity), // does not work
      Vector2f(std::numeric_limits<float>::min(),
               std::numeric_limits<float>::min()),
      [](const Vector2f &top_right, const T &body) {
        return Vector2f(std::max(top_right.x(), body.m_position.x()),
                        std::max(top_right.y(), body.m_position.y()));
      });

  return {bottom_left, top_right};
}

template <typename T, typename = typename std::enable_if<
                          std::is_base_of<Body, T>::value, T>::type>
AlignedBox2f compute_square_bounding_box(const std::vector<T> &bodies) {
  AlignedBox2f min_bbox = compute_minimum_bounding_box(bodies);
  Vector2f bottom_left = min_bbox.min();
  Vector2f top_right = min_bbox.max();
  Vector2f top_left(bottom_left.x(), top_right.y());
  Vector2f bottom_right(top_right.x(), bottom_left.y());

  // Usually, the minimum bounding box is a rectangle. In this case, we have to
  // convert it into a square.
  float width = (top_right - top_left).norm();
  float height = (top_left - bottom_left).norm();
  if (width > height) {
    float diff = width - height;
    bottom_left -= Vector2f(0, diff / 2);
    bottom_right -= Vector2f(0, diff / 2);
    top_right += Vector2f(0, diff / 2);
    top_left += Vector2f(0, diff / 2);
  } else if (height > width) {
    float diff = height - width;
    bottom_left -= Vector2f(diff / 2, 0);
    top_left -= Vector2f(diff / 2, 0);
    bottom_right += Vector2f(diff / 2, 0);
    top_right += Vector2f(diff / 2, 0);
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
    float diff = width - height;
    top_right += Vector2f(0, diff);
    top_left += Vector2f(0, diff);
  } else if (height > width) {
    float diff = height - width;
    top_right += Vector2f(diff, 0);
    bottom_right += Vector2f(diff, 0);
  }

  return {bottom_left, top_right};
}
}  // namespace bh

#endif  // BARNES_HUT_BODY_H
