#ifndef NDEBUG
#include <iostream>
#endif

#include "force.h"
#include "simulated_body.h"

namespace bh {

SimulatedBody::SimulatedBody(Vector2d position, double mass, Vector2d velocity)
    : Body(std::move(position), mass), m_velocity(std::move(velocity)) {}

// Copy constructor
#ifdef DEBUG_COPY_CONSTRUCTOR
SimulatedBody::SimulatedBody(const SimulatedBody &other)
    : Body(other), m_velocity(other.m_velocity) {
  std::cout << "SimulatedBody copy constructor\n";
}
#else
SimulatedBody::SimulatedBody(const SimulatedBody &other) = default;
#endif

// Move constructor
#ifdef DEBUG_MOVE_CONSTRUCTOR
SimulatedBody::SimulatedBody(SimulatedBody &&other) noexcept
    : Body(std::move(other)), m_velocity(std::move(other.m_velocity)) {
  std::cout << "SimulatedBody move constructor\n";
}
#else
SimulatedBody::SimulatedBody(SimulatedBody &&other) noexcept = default;
#endif

// Copy assignment operator
#ifdef DEBUG_COPY_ASSIGNMENT_OPERATOR
SimulatedBody &SimulatedBody::operator=(const SimulatedBody &other) {
  Body::operator=(other);
  std::cout << "SimulatedBody copy assignment operator\n";
  m_velocity = other.m_velocity;
  return *this;
}
#else
SimulatedBody &SimulatedBody::operator=(const SimulatedBody &other) = default;
#endif

// Move assignment operator
#ifdef DEBUG_MOVE_ASSIGNMENT_OPERATOR
SimulatedBody &SimulatedBody::operator=(SimulatedBody &&other) noexcept {
  Body::operator=(std::move(other));
  std::cout << "SimulatedBody move assignment operator\n";
  m_velocity = std::move(other.m_velocity);  // NOLINT(bugprone-use-after-move)
  return *this;
}
#else
SimulatedBody &SimulatedBody::operator=(SimulatedBody &&other) noexcept =
    default;
#endif

SimulatedBody SimulatedBody::updated(const std::vector<SimulatedBody> &bodies,
                                     double dt) const {
  // The body's position is updated according to its current velocity
  Vector2d position(m_position + m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2d force = compute_exact_net_force_on_body(bodies, *this);

  // The body's velocity is updated according to the net force on that
  // particle.
  Vector2d velocity(m_velocity + force / m_mass * dt);

  return {position, m_mass, velocity};
}

SimulatedBody SimulatedBody::updated(const bh::Node &quadtree,
                                     double dt) const {
  // The body's position is updated according to its current velocity
  Vector2d position(m_position + m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2d force = compute_approximate_net_force_on_body(quadtree, *this);

  // The body's velocity is updated according to the net force on that particle.
  Vector2d velocity(m_velocity + force / m_mass * dt);

  return {position, m_mass, velocity};
}

}  // namespace bh