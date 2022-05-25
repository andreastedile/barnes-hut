#ifndef BARNES_HUT_BODY_H
#define BARNES_HUT_BODY_H

#include <eigen3/Eigen/Eigen>

namespace bh {

struct Body {
  Eigen::Vector2f m_position;
  float m_mass;
  Body(Eigen::Vector2f position, float mass);
};

}  // namespace bh

#endif  // BARNES_HUT_BODY_H
