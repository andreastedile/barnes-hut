#ifndef BARNES_HUT_SIMULATION_STEP_H
#define BARNES_HUT_SIMULATION_STEP_H

#include <eigen3/Eigen/Geometry>
#include <vector>

#include "node.h"
#include "body.h"

namespace bh {

struct SimulationStep {
 public:
  std::vector<Body> m_bodies;
  Eigen::AlignedBox2d m_bbox;

  explicit SimulationStep(std::vector<Body> bodies,
                          Eigen::AlignedBox2d bbox);
  // Copy constructor
  SimulationStep(const SimulationStep &other);
  // Move constructor
  SimulationStep(SimulationStep &&other) noexcept;
  // Copy assignment operator
  SimulationStep &operator=(const SimulationStep &other);
  // Move assignment operator
  SimulationStep &operator=(SimulationStep &&other) noexcept;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_STEP_H
