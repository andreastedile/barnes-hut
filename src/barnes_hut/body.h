#ifndef BARNES_HUT_BODY_H
#define BARNES_HUT_BODY_H

#include <eigen3/Eigen/Eigen>
#include <nlohmann/json.hpp>
#include <vector>

using Eigen::Vector2d;
using nlohmann::json;

namespace bh {

/**
 * A point particle in the cartesian plane
 */
struct Body {
  // Position vector of the body in the cartesian plane
  Vector2d m_position;
  // Mass of the body
  double m_mass;
  // Velocity vector of the body
  Vector2d m_velocity;

  Body();
  /**
   * Creates a body with given position and mass, and zero velocity.
   * @param position of the body in the cartesian plane
   * @param mass of the body
   */
  Body(Vector2d position, double mass);
  /**
   * Creates a body with given position, mass and velocity.
   * @param position of the body in the cartesian plane
   * @param mass of the body
   * @param velocity of the body
   */
  Body(Vector2d position, double mass, Vector2d velocity);

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS
  Body(const Body &other);
  Body(Body &&other) noexcept;
  Body &operator=(const Body &other);
  Body &operator=(Body &&other) noexcept;
#endif
};

void to_json(json &j, const Body &body);

}  // namespace bh

#endif  // BARNES_HUT_BODY_H
