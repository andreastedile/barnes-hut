#include "body.h"

#include <utility>  // move
#ifndef NDEBUG
#include <iostream>
#endif

namespace bh {

Body::Body(Vector2d position, double mass)
    : m_position(std::move(position)), m_mass(mass) {}

// Copy constructor
#ifdef DEBUG_COPY_CONSTRUCTOR
Body::Body(const Body &other)
    : m_position(other.m_position), m_mass(other.m_mass) {
  std::cout << "Body copy constructor\n";
}
#else
Body::Body(const Body &other) = default;
#endif

// Move constructor
#ifdef DEBUG_MOVE_CONSTRUCTOR
Body::Body(Body &&other) noexcept
    : m_position(std::move(other.m_position)), m_mass(other.m_mass) {
  std::cout << "Body move constructor\n";
}
#else
Body::Body(Body &&other) noexcept = default;
#endif

// Copy assignment operator
#ifdef DEBUG_COPY_ASSIGNMENT_OPERATOR
Body &Body::operator=(const Body &other) {
  std::cout << "Body copy assignment operator\n";
  m_position = other.m_position;
  m_mass = other.m_mass;
  return *this;
}
#else
Body &Body::operator=(const Body &other) = default;
#endif

// Move assignment operator
#ifdef DEBUG_MOVE_ASSIGNMENT_OPERATOR
Body &Body::operator=(Body &&other) noexcept {
  std::cout << "Body move assignment operator\n";
  m_position = std::move(m_position);
  m_mass = other.m_mass;
  return *this;
}
#else
Body &Body::operator=(Body &&other) noexcept = default;
#endif

}  // namespace bh
