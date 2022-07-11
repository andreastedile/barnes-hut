#include "force.h"

#ifdef WITH_TBB
#include <execution>  // par_unseq
#else
// https://passlab.github.io/Examples/contents/Examples_udr.html
// https://www.openmp.org/spec-html/5.0/openmpsu107.html
// clang-format off
#pragma omp declare reduction(+ : Eigen::Vector2d : \
                              omp_out += omp_in) \
        initializer(omp_priv = {0, 0})
// clang-format on
#endif
#include <numeric>  // accumulate, transform_reduce
#include <variant>  // visit
#ifdef DEBUG_COMPUTE_GRAVITATIONAL_FORCE
#include <spdlog/spdlog.h>
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
    spdlog::trace("distance: {}", distance);
    spdlog::trace("magnitude: {}", magnitude);
    spdlog::trace("angle: {}", angle);
    spdlog::trace("fx: {}", fx);
    spdlog::trace("fy: {}", fy);
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
      return compute_approximate_net_force_on_body(*fork.m_nw, body, G, omega) +
             compute_approximate_net_force_on_body(*fork.m_ne, body, G, omega) +
             compute_approximate_net_force_on_body(*fork.m_se, body, G, omega) +
             compute_approximate_net_force_on_body(*fork.m_sw, body, G, omega);
    }
  };

  return std::visit(overloaded{visit_fork, visit_leaf}, node.data());
}

Eigen::Vector2d compute_exact_net_force_on_body_parallel(const std::vector<Body>& bodies, const Body& body,
                                                         double G) {
#ifdef WITH_TBB
  return std::transform_reduce(
      std::execution::par_unseq, bodies.begin(), bodies.end(), Eigen::Vector2d{0, 0},
      [](const Eigen::Vector2d& total, const Eigen::Vector2d& curr) {
        return (total + curr).eval();
      },
      [&](const Body& curr) {
        return compute_gravitational_force(curr, body, G);
      });
#else
  Eigen::Vector2d net_force{0, 0};

// clang-format off
#pragma omp parallel for default(none) shared(bodies, body, G) reduction(+ : net_force)
    for (const auto &curr : bodies) {
      net_force += compute_gravitational_force(curr, body, G);
    }
// clang-format off

    return net_force;
#endif
}

Eigen::Vector2d compute_exact_net_force_on_body_serial(const std::vector<Body>& bodies, const Body& body,
                                                double G) {
  return std::transform_reduce(bodies.begin(), bodies.end(),
      Eigen::Vector2d{0, 0},
      [](const Eigen::Vector2d& total, const Eigen::Vector2d& curr) {
        return (total + curr).eval();
      },
      [&](const Body& curr) {
        return compute_gravitational_force(curr, body, G);
      });
}


}  // namespace bh
