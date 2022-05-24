#include "barnes_hut/simulation.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "barnes_hut/force.h"
#include "barnes_hut/node.h"

namespace bh {

SimulatedBody SimulatedBody::updated(const bh::Node& quadtree, float dt) {
  // The body's position is updated according to its current velocity
  Vector2f position(m_position + m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2f force = bh::compute_approximate_net_force_on_body(quadtree, *this);

  // The body's velocity is updated according to the net force on that particle.
  Vector2f velocity(m_velocity + force / m_mass * dt);

  return {position, m_mass, velocity};
}

}  // namespace bh
