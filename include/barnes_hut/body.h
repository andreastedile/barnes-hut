#ifndef BARNES_HUT_BODY_H
#define BARNES_HUT_BODY_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <numeric>
#include <type_traits>

using Eigen::AlignedBox2f;
using Eigen::Vector2f;

namespace bh {

struct Body {
  Vector2f m_position;
  float m_mass;
  Body(Vector2f position, float mass);
};

// We implement the function in the header itself.
// https://stackoverflow.com/questions/10632251/undefined-reference-to-template-function
template <typename T, typename = typename std::enable_if<
                          std::is_base_of<Body, T>::value, T>::type>
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

  // We want a square bounding box
  Eigen::Vector2f top_left(bottom_left.x(), top_right.y());
  Eigen::Vector2f bottom_right(top_right.x(), bottom_left.y());

  float width = (top_right - top_left).norm();
  float height = (top_left - bottom_left).norm();
  if (width > height) {
    float diff = width - height;
    bottom_left = {bottom_left.x(), bottom_left.y() - diff / 2};
    top_right = {top_right.x(), top_right.y() + diff / 2};
  } else if (height > width) {
    float diff = height - width;
    bottom_left = {bottom_left.x() - diff / 2, bottom_left.y()};
    top_right = {top_right.x() + diff / 2, top_right.y()};
  }

  return {bottom_left, top_right};
}
}  // namespace bh

#endif  // BARNES_HUT_BODY_H
