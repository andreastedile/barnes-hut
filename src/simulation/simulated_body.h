#ifndef BARNES_HUT_SIMULATED_BODY_H
#define BARNES_HUT_SIMULATED_BODY_H

#include <eigen3/Eigen/Eigen>
#include <vector>

#include "body.h"
#include "node.h"

using Eigen::Vector2d;

namespace bh {

/**
 * A point particle in the cartesian plane, with an additional field
 * representing its velocity vector.
 */
struct SimulatedBody final : Body {
  // Velocity vector of the body
  Vector2d m_velocity;

  /**
   * Creates a body with given position, velocity and mass.
   * @param position of the body in the cartesian plane
   * @param mass of the body
   * @param velocity of the body
   */
  SimulatedBody(Vector2d position, double mass, Vector2d velocity);

  /**
   * Computes the new position and velocity vectors of the body after a
   * simulation step, using the Barnes–Hut approximation algorithm.
   * @param quadtree containing a set of bodies that exert a gravitational force
   * on this body
   * @param dt simulation timestep; defines the accuracy of the computation: the
   * smaller, the more accurate
   * @return a new body containing the updated position and velocity vectors
   */
  [[nodiscard]] SimulatedBody updated(const bh::Node &quadtree,
                                      double dt) const;

  /**
   * Computes the new and exact position and velocity vectors of the body after
   * a simulation step.
   * @param bodies that exert a gravitational force on this body
   * @param dt simulation timestep; defines the accuracy of the computation: the
   * smaller, the more accurate
   * @return a new body containing the updated position and velocity vectors
   */
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
