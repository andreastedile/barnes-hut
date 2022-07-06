#include <execution>  // par_unseq
#include <numeric>    // accumulate, transform_reduce
#include <variant>    // visit
#ifdef DEBUG_COMPUTE_GRAVITATIONAL_FORCE
#include <iostream>
#endif
#include <cmath>

#include "../quadtree/node.h"

// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

namespace bh {

Eigen::Vector2d compute_gravitational_force(const Body& b1, const Body& b2, double G) {
  Eigen::Vector2d direction_v = b1.m_position - b2.m_position;
  if (double distance = direction_v.norm(); distance > 0) {
    double magnitude = (G * b1.m_mass * b2.m_mass) / std::pow(distance, 2.f);
    double angle = std::atan2(direction_v.y(), direction_v.x());
    double fx = std::cos(angle) * magnitude;
    double fy = std::sin(angle) * magnitude;
#ifdef DEBUG_COMPUTE_GRAVITATIONAL_FORCE
    std::cout << "distance: " << distance << '\n';
    std::cout << "magnitude: " << magnitude << '\n';
    std::cout << "angle: " << angle << '\n';
    std::cout << "fx: " << fx << '\n';
    std::cout << "fy: " << fy << '\n';
#endif
    return {fx, fy};
  } else {
    return {0, 0};
  }
}

Eigen::Vector2d compute_approximate_net_force_on_body(const Node& node, const Body& body,
                                                      double G, double omega) {
  const auto visit_leaf = [&](const Node::Leaf& leaf) -> Eigen::Vector2d {
    if (leaf.m_body.has_value()) {
      return compute_gravitational_force(*leaf.m_body, body, G);
    }
    return {0, 0};
  };

  const auto visit_fork = [&](const Node::Fork& fork) -> Eigen::Vector2d {
    if (double distance = (body.m_position - node.center_of_mass()).norm();
        node.bbox().sizes().x() / distance < omega) {
      // Approximation
      return bh::compute_gravitational_force(
          {node.center_of_mass(), node.total_mass()}, body, G);
    } else {
      return std::accumulate(
          fork.m_children.begin(), fork.m_children.end(), Eigen::Vector2d{0, 0},
          [&](const Eigen::Vector2d& total, const std::unique_ptr<Node>& curr) {
            return (total + compute_approximate_net_force_on_body(*curr, body, G, omega))
                .eval();
          });
    }
  };

  return std::visit(overloaded{visit_fork, visit_leaf}, node.data());
}

Eigen::Vector2d compute_exact_net_force_on_body(const std::vector<Body>& bodies, const Body& body,
                                                double G) {
  return std::transform_reduce(
      std::execution::par_unseq, bodies.begin(), bodies.end(), Eigen::Vector2d{0, 0},
      [](const Eigen::Vector2d& total, const Eigen::Vector2d& curr) {
        return (total + curr).eval();
      },
      [&](const Body& curr) {
        return compute_gravitational_force(curr, body, G);
      });
}

}  // namespace bh
