#ifndef BARNES_HUT_SIMULATION_STEP_H
#define BARNES_HUT_SIMULATION_STEP_H

#include <eigen3/Eigen/Geometry>  // AlignedBox2d
#include <nlohmann/json.hpp>
#include <vector>

#include "body.h"

namespace bh {

class SimulationStep {
 public:
  SimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox);

  [[nodiscard]] const std::vector<Body> &bodies() const;
  [[nodiscard]] const Eigen::AlignedBox2d &bbox() const;

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS
  SimulationStep(const SimulationStep &other);
  SimulationStep(SimulationStep &&other) noexcept;
  SimulationStep &operator=(const SimulationStep &other);
  SimulationStep &operator=(SimulationStep &&other) noexcept;
#endif

 protected:  // and not private, so that they can be moved
  std::vector<Body> m_bodies;
  Eigen::AlignedBox2d m_bbox;
};

void to_json(nlohmann::json &j, const SimulationStep &step);

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_STEP_H
