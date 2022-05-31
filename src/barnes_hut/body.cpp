#include "body.h"

#include <utility>  // move

namespace bh {

Body::Body(Vector2d position, double mass)
    : m_position(std::move(position)), m_mass(mass) {}

}  // namespace bh
