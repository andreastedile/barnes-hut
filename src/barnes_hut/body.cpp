#include "body.h"

#include <utility>  // move

namespace bh {

Body::Body(Vector2f position, float mass)
    : m_position(std::move(position)), m_mass(mass) {}

}  // namespace bh
