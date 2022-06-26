#include "local_exact_simulator.h"

#include <fstream>  // ofstream
#ifndef NDEBUG
#include <iomanip>  // setw
#endif
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
  update_max_bbox(new_bbox);
  auto simulation_step = std::make_shared<ExactSimulationStep>(std::move(new_bodies), std::move(new_bbox));
  m_simulation_steps.push_back(simulation_step);
  return simulation_step;
}

json LocalExactSimulator::to_json() const { return *this; }

void LocalExactSimulator::save() const {
  std::ofstream o("simulation.json");
#ifndef NDEBUG
  o << std::setw(2) << to_json();
#else
  o << to_json();
#endif
}

}  // namespace bh