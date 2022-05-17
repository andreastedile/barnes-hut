#ifndef BARNES_HUT_BODY_H
#define BARNES_HUT_BODY_H

#include <eigen3/Eigen/Eigen>

struct Body {
  Eigen::Vector2f m_position;
  float m_mass;
  Body(const Eigen::Vector2f& position, float mass);
};

#endif  // BARNES_HUT_BODY_H
