#ifndef BARNES_HUT_BARNES_HUT_SIMULATION_STEP_H
#define BARNES_HUT_BARNES_HUT_SIMULATION_STEP_H

#include <memory>  // shared_ptr
#include <nlohmann/json.hpp>

#include "node.h"
#include "simulation_step.h"

namespace bh {

class BarnesHutSimulationStep final : public SimulationStep {
 public:
  BarnesHutSimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox);
  BarnesHutSimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox, std::shared_ptr<const Node> quadtree);

  [[nodiscard]] const Node &quadtree() const;

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS
  BarnesHutSimulationStep(const BarnesHutSimulationStep &other);
  BarnesHutSimulationStep(BarnesHutSimulationStep &&other) noexcept;
  BarnesHutSimulationStep &operator=(const BarnesHutSimulationStep &other);
  BarnesHutSimulationStep &operator=(BarnesHutSimulationStep &&other) noexcept;
#endif

 protected:  // and not private, so that they can be moved
  std::shared_ptr<const Node> m_quadtree;
};

void to_json(nlohmann::json &j, const BarnesHutSimulationStep &step);

}  // namespace bh

#endif  // BARNES_HUT_BARNES_HUT_SIMULATION_STEP_H
