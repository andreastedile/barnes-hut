#ifndef BARNES_HUT_FORCE_H
#define BARNES_HUT_FORCE_H

#include "body.h"
#include "node.h"

namespace bh::force {

/**
 * @return Force that body b1 exerts on body b2
 */
Eigen::Vector2f compute_gravitational_force(const Body& b1, const Body& b2);

Eigen::Vector2f compute_approximate_net_force_on_body(const Node& node,
                                                      const Body& body);

Eigen::Vector2f compute_exact_net_force_on_body(const Node& node,
                                                const Body& body);

}  // namespace bh::force

#endif  // BARNES_HUT_FORCE_H
