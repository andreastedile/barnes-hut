#ifndef BARNES_HUT_SIMULATION_STEP_H
#define BARNES_HUT_SIMULATION_STEP_H

#include <eigen3/Eigen/Geometry>
#include <vector>

#include "body.h"

using Eigen::AlignedBox2d;

namespace bh {

class SimulationStep {
 public:
  SimulationStep(std::vector<Body> bodies, const AlignedBox2d &bbox);

  [[nodiscard]] const std::vector<Body> &bodies() const;
  [[nodiscard]] const AlignedBox2d &bbox() const;

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS
  SimulationStep(const SimulationStep &other);
  SimulationStep(SimulationStep &&other) noexcept;
  SimulationStep &operator=(const SimulationStep &other);
  SimulationStep &operator=(SimulationStep &&other) noexcept;
#endif

 protected:
  std::vector<Body> m_bodies;
  AlignedBox2d m_bbox;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_STEP_H
