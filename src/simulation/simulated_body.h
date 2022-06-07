#ifndef BARNES_HUT_SIMULATED_BODY_H
#define BARNES_HUT_SIMULATED_BODY_H

#include <eigen3/Eigen/Eigen>
#include <vector>

#include "body.h"
#include "node.h"

using Eigen::Vector2d;

namespace bh {

struct SimulatedBody : Body {
  Vector2d m_velocity;

  SimulatedBody(Vector2d position, double mass, Vector2d velocity);

  [[nodiscard]] SimulatedBody updated(const bh::Node &quadtree,
                                      double dt) const;

  [[nodiscard]] SimulatedBody updated(const std::vector<SimulatedBody> &bodies,
                                      double dt) const;

  // Copy constructor
  SimulatedBody(const SimulatedBody &other);
  // Move constructor
  SimulatedBody(SimulatedBody &&other) noexcept;
  // Copy assignment operator
  SimulatedBody &operator=(const SimulatedBody &other);
  // Move assignment operator
  SimulatedBody &operator=(SimulatedBody &&other) noexcept;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATED_BODY_H
