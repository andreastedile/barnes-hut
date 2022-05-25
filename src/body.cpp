#include "barnes_hut/body.h"

#include <utility>

namespace bh {

Body::Body(Eigen::Vector2f position, float mass)
    : m_position(std::move(position)), m_mass(mass) {}

}  // namespace bh
