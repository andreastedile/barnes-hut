#ifndef BARNES_HUT_FORCE_H
#define BARNES_HUT_FORCE_H

#include <eigen3/Eigen/Eigen>
#include <type_traits>  // enable_if, is_base_of

#include "body.h"
#include "execution"
#include "node.h"

using Eigen::Vector2d;

namespace bh {

/**
 * @return Force that body b1 exerts on body b2
 */
Vector2d compute_gravitational_force(const Body& b1, const Body& b2);

Vector2d compute_approximate_net_force_on_body(const Node& node,
                                               const Body& body);

template <typename T, typename = typename std::enable_if<
                          std::is_base_of<Body, T>::value, T>::type>
Vector2d compute_exact_net_force_on_body(const std::vector<T>& bodies,
                                         const T& body) {
  return std::reduce(
      std::execution::par_unseq, bodies.begin(), bodies.end(), Vector2d{0, 0},
      [&body](const Vector2d& total, const T& curr) {
        return (total + compute_gravitational_force(curr, body)).eval();
      });
}

}  // namespace bh

#endif  // BARNES_HUT_FORCE_H
