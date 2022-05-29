#ifndef BARNES_HUT_FORCE_H
#define BARNES_HUT_FORCE_H

#include <eigen3/Eigen/Eigen>

#include "body.h"
#include "node.h"

using Eigen::Vector2f;

namespace bh {

/**
 * @return Force that body b1 exerts on body b2
 */
Vector2f compute_gravitational_force(const Body& b1, const Body& b2);

Vector2f compute_approximate_net_force_on_body(const Node& node,
                                               const Body& body);

Vector2f compute_exact_net_force_on_body(const Node& node, const Body& body);

}  // namespace bh

#endif  // BARNES_HUT_FORCE_H
