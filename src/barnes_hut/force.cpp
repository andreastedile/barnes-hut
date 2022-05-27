#include <iostream>
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

Eigen::Vector2f compute_gravitational_force(const Body& b1, const Body& b2) {
  Eigen::Vector2f direction_v = b1.m_position - b2.m_position;
  float distance = direction_v.norm();
  if (distance == 0) {
    return {0, 0};
  }
  float magnitude = (G * b1.m_mass * b2.m_mass) / std::pow(distance, 2.f);
  float angle = std::atan2(direction_v.y(), direction_v.x());
  float fx = std::cos(angle) * magnitude;
  float fy = std::sin(angle) * magnitude;
  Eigen::Vector2f force_v(fx, fy);
  return force_v;
}

Eigen::Vector2f compute_approximate_net_force_on_body(const Node& node,
                                                      const Body& body) {
  const auto visit_empty = [](const Empty&) { return Eigen::Vector2f(0, 0); };

  const auto visit_body = [body](const Body& visited) {
    return compute_gravitational_force(visited, body);
  };

  const auto visit_region = [&](const Subquadrants& subquadrants) {
    float distance = (body.m_position - node.center_of_mass()).norm();
    if (node.length() / distance < OMEGA) {
      // Approximation
      return bh::compute_gravitational_force(
          {node.center_of_mass(), node.total_mass()}, body);
    } else {
      return std::accumulate(
          subquadrants.begin(), subquadrants.end(), Eigen::Vector2f(0, 0),
          [body](const Eigen::Vector2f& total,
                 const std::unique_ptr<Node>& curr) {
            return (total + compute_approximate_net_force_on_body(*curr, body))
                .eval();
          });
    }
  };

  return std::visit(overloaded{visit_empty, visit_body, visit_region},
                    node.data());
}

Eigen::Vector2f compute_exact_net_force_on_body(const Node& node,
                                                const Body& body) {
  const auto visit_empty = [](const Empty&) { return Eigen::Vector2f(0, 0); };

  const auto visit_body = [body](const Body& visited) {
    return compute_gravitational_force(visited, body);
  };

  const auto visit_region = [&](const Subquadrants& subquadrants) {
    return std::accumulate(
        subquadrants.begin(), subquadrants.end(), Eigen::Vector2f(0, 0),
        [body](const Eigen::Vector2f& total,
               const std::unique_ptr<Node>& curr) {
          return (total + compute_approximate_net_force_on_body(*curr, body))
              .eval();
        });
  };

  return std::visit(overloaded{visit_empty, visit_body, visit_region},
                    node.data());
}

}  // namespace bh::force
