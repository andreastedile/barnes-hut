#include <numeric>  // accumulate
#include <variant>

#include "node.h"
#include "templates.h"

namespace bh {

#ifndef G
// https://physics.nist.gov/cgi-bin/cuu/Value?bg
// 6.674 30 x 10-11 m3 kg-1 s-2
// #define G 0.000000000066743f
#define G 0.5f
#endif

#ifndef OMEGA
#define OMEGA 0.5f
#endif

Vector2d compute_gravitational_force(const Body& b1, const Body& b2) {
  Vector2d direction_v = b1.m_position - b2.m_position;
  double distance = direction_v.norm();
  if (distance == 0) {
    return {0, 0};
  }
  double magnitude = (G * b1.m_mass * b2.m_mass) / std::pow(distance, 2.f);
  double angle = std::atan2(direction_v.y(), direction_v.x());
  double fx = std::cos(angle) * magnitude;
  double fy = std::sin(angle) * magnitude;
  return {fx, fy};
}

Vector2d compute_approximate_net_force_on_body(const Node& node,
                                               const Body& body) {
  const auto visit_leaf = [&](const Node::Leaf& leaf) -> Vector2d {
    if (leaf.m_body.has_value()) {
      return compute_gravitational_force(*leaf.m_body, body);
    }
    return {0, 0};
  };

  const auto visit_fork = [&](const Node::Fork& fork) -> Vector2d {
    if (double distance = (body.m_position - node.center_of_mass()).norm();
        node.length() / distance < OMEGA) {
      // Approximation
      return bh::compute_gravitational_force(
          {node.center_of_mass(), node.total_mass()}, body);
    } else {
      return std::accumulate(
          fork.m_children.begin(), fork.m_children.end(), Vector2d{0, 0},
          [&body](const Vector2d& total, const std::unique_ptr<Node>& curr) {
            return (total + compute_approximate_net_force_on_body(*curr, body))
                .eval();
          });
    }
  };

  return std::visit(overloaded{visit_fork, visit_leaf}, node.data());
}

Vector2d compute_exact_net_force_on_body(const Node& node, const Body& body) {
  const auto visit_leaf = [&](const Node::Leaf& leaf) -> Vector2d {
    if (leaf.m_body.has_value()) {
      return compute_gravitational_force(*leaf.m_body, body);
    }
    return {0, 0};
  };

  const auto visit_fork = [&](const Node::Fork& fork) -> Vector2d {
    return std::accumulate(
        fork.m_children.begin(), fork.m_children.end(), Vector2d{0, 0},
        [&body](const Vector2d& total, const std::unique_ptr<Node>& curr) {
          return (total + compute_approximate_net_force_on_body(*curr, body))
              .eval();
        });
  };

  return std::visit(overloaded{visit_fork, visit_leaf}, node.data());
}

}  // namespace bh
