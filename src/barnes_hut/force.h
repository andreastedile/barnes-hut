#ifndef BARNES_HUT_FORCE_H
#define BARNES_HUT_FORCE_H

#include <eigen3/Eigen/Eigen>

#include "body.h"
#include "node.h"

using Eigen::Vector2d;

namespace bh {

// https://physics.nist.gov/cgi-bin/cuu/Value?bg
// 6.674 30 x 10-11 m3 kg-1 s-2
// = 0.000000000066743
constexpr double NEWTONIAN_G = 0.000000000066743;

constexpr double DEFAULT_OMEGA = 0.5;

/**
 * Computes the gravitational force that body b1 exerts on body b2.
 * @details If the two bodies coincide, the components of the resulting force
 * vector are (0, 0)
 * @param b1 exerts a gravitational force on b2
 * @param b2 is subject to the gravitational force of b1
 * @return A force vector
 */
Vector2d compute_gravitational_force(const Body& b1, const Body& b2,
                                     double G = NEWTONIAN_G);

/**
 * Computes the net gravitational force that a set of bodies contained in the
 * node of a quadtree exert on a body, using the Barnes–Hut approximation
 * algorithm.
 * @param node of a quadtree; can be the root, a fork or a leaf.
 * @param body that is subject to the gravitational force of the bodies in node
 * @return A force vector
 */
Vector2d compute_approximate_net_force_on_body(const Node& node, const Body& body,
                                               double G = NEWTONIAN_G, double omega = DEFAULT_OMEGA);

/**
 * Computes the exact gravitational force that a set of bodies exert on a body,
 * without any approximation.
 * @param bodies that exert a gravitational force on body
 * @param body that is subject to the gravitational force of bodies
 * @return A force vector
 */
Vector2d compute_exact_net_force_on_body(const std::vector<Body>& bodies, const Body& body,
                                         double G = NEWTONIAN_G);

}  // namespace bh

#endif  // BARNES_HUT_FORCE_H
