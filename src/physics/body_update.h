#ifndef BARNES_HUT_BODY_UPDATE_H
#define BARNES_HUT_BODY_UPDATE_H

#include <vector>

#include "body.h"
#include "node.h"

namespace bh {

/**
 * Computes the new and exact position and velocity vectors of the body after a simulation step_impl.
 * @param bodies that exert a gravitational force on this body
 * @param dt simulation timestep; defines the accuracy of the computation: the  smaller, the more accurate
 * @return a new body containing the updated position and velocity vectors
 */
Body update_body(const Body& body, const std::vector<Body>& bodies, double dt, double G);

/**
 * Computes the new, approximated position and velocity vectors of the body after a simulation step_impl,
 * using the Barnes–Hut approximation algorithm.
 * @param quadtree containing a set of bodies that exert a gravitational force on this body
 * @param dt simulation timestep; defines the accuracy of the computation: the smaller, the more accurate
 * @return a new body containing the updated position and velocity vectors
 */
Body update_body(const Body& body, const Node& quadtree, double dt, double G, double omega);

}  // namespace bh

#endif  // BARNES_HUT_BODY_UPDATE_H
