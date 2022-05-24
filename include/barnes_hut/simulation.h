#ifndef BARNES_HUT_SIMULATION_H
#define BARNES_HUT_SIMULATION_H

#include <Eigen/Eigen>
#include <utility>

#include "body.h"
#include "node.h"

using Eigen::Vector2f;

namespace bh {

struct SimulatedBody : Body {
  Eigen::Vector2f m_velocity;
  SimulatedBody(const Eigen::Vector2f& position, float mass,
                Eigen::Vector2f velocity)
      : Body(position, mass), m_velocity(std::move(velocity)) {}
  SimulatedBody updated(const bh::Node& quadtree, float dt);
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_H
