#ifndef BARNES_HUT_BARNES_HUT_SIMULATION_STEP_H
#define BARNES_HUT_BARNES_HUT_SIMULATION_STEP_H

#include <eigen3/Eigen/Geometry>
#include <memory>  // shared_ptr

#include "simulation_step.h"

namespace bh {

struct BarnesHutSimulationStep final : public SimulationStep {
  std::shared_ptr<const Node> m_quadtree;

  BarnesHutSimulationStep(std::vector<SimulatedBody> bodies,
                          Eigen::AlignedBox2d bbox,
                          std::shared_ptr<const Node> quadtree);
  // Copy constructor
  BarnesHutSimulationStep(const BarnesHutSimulationStep &other);
  // Move constructor
  BarnesHutSimulationStep(BarnesHutSimulationStep &&other) noexcept;
  // Copy assignment operator
  BarnesHutSimulationStep &operator=(const BarnesHutSimulationStep &other);
  // Move assignment operator
  BarnesHutSimulationStep &operator=(BarnesHutSimulationStep &&other) noexcept;
};

void to_json(json &j, const BarnesHutSimulationStep &step);

}  // namespace bh

#endif  // BARNES_HUT_BARNES_HUT_SIMULATION_STEP_H
