#include "body.h"

#include <utility>  // move
#ifndef NDEBUG
#include <iostream>
#endif

namespace bh {

Body::Body() : m_position{0, 0}, m_mass(0) {}

Body::Body(Vector2d position, double mass)
    : m_position(std::move(position)), m_mass(mass), m_velocity{0, 0} {}

Body::Body(Vector2d position, double mass, Vector2d velocity)
    : m_position(std::move(position)), m_mass(mass), m_velocity(std::move(velocity)) {}

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS

Body::Body(const Body &other)
    : m_position(other.m_position), m_mass(other.m_mass), m_velocity(other.m_velocity) {
  std::cout << "Body copy constructor\n";
}

Body::Body(Body &&other) noexcept
    : m_position(std::move(other.m_position)), m_mass(other.m_mass), m_velocity(std::move(other.m_velocity)) {
  std::cout << "Body move constructor\n";
}

Body &Body::operator=(const Body &other) {
  std::cout << "Body copy assignment operator\n";
  m_position = other.m_position;
  m_mass = other.m_mass;
  m_velocity = other.m_velocity;
  return *this;
}

Body &Body::operator=(Body &&other) noexcept {
  std::cout << "Body move assignment operator\n";
  m_position = std::move(other.m_position);
  m_mass = other.m_mass;
  m_velocity = std::move(other.m_velocity);
  return *this;
}

#endif

}  // namespace bh
