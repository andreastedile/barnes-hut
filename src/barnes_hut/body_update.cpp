#include "body_update.h"

#include "force.h"

namespace bh {

Body update_body(const Body& body,
                 const std::vector<Body>& bodies,
                 double dt, double G) {
  // The body's position is updated according to its current velocity
  Vector2d position(body.m_position + body.m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2d force = compute_exact_net_force_on_body(bodies, body, G);

  // The body's velocity is updated according to the net force on that
  // particle.
  Vector2d velocity(body.m_velocity + force / body.m_mass * dt);

  return {position, body.m_mass, velocity};
}

Body update_body(const Body& body,
                 const Node& quadtree,
                 double dt, double G, double omega) {
  // The body's position is updated according to its current velocity
  Vector2d position(body.m_position + body.m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2d force = compute_approximate_net_force_on_body(quadtree, body, G, omega);

  // The body's velocity is updated according to the net force on that particle.
  Vector2d velocity(body.m_velocity + force / body.m_mass * dt);

  return {position, body.m_mass, velocity};
}

}  // namespace bh