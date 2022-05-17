#include "barnes_hut/body.h"

Body::Body(const Eigen::Vector2f& position, float mass)
    : m_position(position), m_mass(mass) {}
