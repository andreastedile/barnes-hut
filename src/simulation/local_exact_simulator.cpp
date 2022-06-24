#include "local_exact_simulator.h"

#include <memory>   // make_shared
#include <utility>  // move

#include "body.h"  // compute_square_bounding_box
#include "exact_simulation_step.h"

namespace bh {

LocalExactSimulator::LocalExactSimulator(const std::string& filename, double dt, double G, double omega)
    : ISimulation(dt, G, omega) {
  auto bodies = load(filename);
  auto bbox = compute_square_bounding_box(bodies);
  auto simulation_step = std::make_shared<ExactSimulationStep>(std::move(bodies), std::move(bbox));
  m_simulation_steps.push_back(std::move(simulation_step));
}

std::shared_ptr<SimulationStep> LocalExactSimulator::step() {
  auto [new_bodies, new_bbox] =
      compute_new_bodies_exact(m_simulation_steps.back()->m_bodies, m_dt, m_G);
  auto simulation_step = std::make_shared<ExactSimulationStep>(std::move(new_bodies), std::move(new_bbox));
  m_simulation_steps.push_back(simulation_step);
  return simulation_step;
}

}  // namespace bh