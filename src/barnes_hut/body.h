#ifndef BARNES_HUT_BODY_H
#define BARNES_HUT_BODY_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <vector>

using Eigen::AlignedBox2d;
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
  // Copy constructor
  Body(const Body &other);
  // Move constructor
  Body(Body &&other) noexcept;
  // Copy assignment operator
  Body &operator=(const Body &other);
  // Move assignment operator
  Body &operator=(Body &&other) noexcept;
};

/**
 * Computes the axis-aligned minimum bounding box containing some bodies.
 * @details The minimum bounding box is defined by:
 * <ul>
 * <li> Its bottom-left corner, whose x and y coordinates are minimal for any of
 * the bodies
 * <li> Its top-right corner, whose x and y coordinates are maximal for any of
 * the bodies
 * </ul>
 * @param bodies for which to compute the minimum bounding box; must be at least
 * two
 * @return the minimum bounding box; x() and y() return its bottom-left and
 * top-right corners
 * @throw invalid_argument if the bodies vector contains less than two bodies
 * @example <a href="https://www.desmos.com/calculator/mintua3fvc?lang=it">on
 * Desmos</a>
 */
AlignedBox2d compute_minimum_bounding_box(const std::vector<Body> &bodies);

/**
 * Computes an axis-aligend square bounding box containing some bodies.
 * @details The bounding box is defined by its bottom-left and top-right
 * corners. The computed bounding box is not minimum, but is
 *
 * @param bodies for which to compute the minimum bounding box; must be at least
 * two
 * @return the minimum bounding box; x() and y() return its bottom-left and
 * top-right corners
 * @throw invalid_argument if the bodies vector contains less than two bodies
 * @example <a href="https://www.desmos.com/calculator/mintua3fvc?lang=it">on
 * Desmos</a>
 */
AlignedBox2d compute_square_bounding_box(const std::vector<Body> &bodies);

void to_json(json &j, const Body &body);

}  // namespace bh

#endif  // BARNES_HUT_BODY_H
