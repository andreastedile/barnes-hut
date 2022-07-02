#include "body.h"

#include <stdexcept>
#include <string>   // to_string
#include <utility>  // move
#ifndef NDEBUG
#include <cstdio>
#endif

namespace bh {

Body::Body() : m_position{0, 0}, m_mass{0}, m_velocity{0, 0} {}

Body::Body(Eigen::Vector2d position, double mass)
    : m_position(std::move(position)), m_mass(mass), m_velocity{0, 0} {}

Body::Body(Eigen::Vector2d position, double mass, Eigen::Vector2d velocity)
    : m_position(std::move(position)), m_mass(mass), m_velocity(std::move(velocity)) {}

std::istream &operator>>(std::istream &stream, Body &body) {
  stream >> body.m_position.x();
  stream >> body.m_position.y();
  stream >> body.m_mass;
  if (body.m_mass < 0) {
    throw std::runtime_error("invalid body mass (read " + std::to_string(body.m_mass) + ", should be non-negative)");
  }
  stream >> body.m_velocity.x();
  stream >> body.m_velocity.y();
  return stream;
}

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS

Body::Body(const Body &other)
    : m_position(other.m_position), m_mass(other.m_mass), m_velocity(other.m_velocity) {
  std::puts("Body copy constructor");
}

Body::Body(Body &&other) noexcept
    : m_position(std::move(other.m_position)), m_mass(other.m_mass), m_velocity(std::move(other.m_velocity)) {
  std::puts("Body move constructor");
}

Body &Body::operator=(const Body &other) {
  std::puts("Body copy assignment operator");
  m_position = other.m_position;
  m_mass = other.m_mass;
  m_velocity = other.m_velocity;
  return *this;
}

Body &Body::operator=(Body &&other) noexcept {
  std::puts("Body move assignment operator");
  m_position = std::move(other.m_position);
  m_mass = other.m_mass;
  m_velocity = std::move(other.m_velocity);
  return *this;
}

#endif

}  // namespace bh
