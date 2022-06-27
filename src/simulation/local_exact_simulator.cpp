#include "local_exact_simulator.h"

#include <fstream>  // ofstream
#ifndef NDEBUG
#include <iomanip>  // setw
#endif
#include <memory>   // make_shared, static_pointer_cast
#include <utility>  // move

#include "body.h"  // compute_square_bounding_box
#include "exact_simulation_step.h"

namespace bh {

LocalExactSimulator::LocalExactSimulator(const std::string& filename, double dt, double G)
    : ISimulation(dt, G) {
  auto bodies = load(filename);
  m_max_bbox = compute_square_bounding_box(bodies);
  m_simulation_steps.push_back(std::make_shared<ExactSimulationStep>(std::move(bodies), m_max_bbox));
}

void LocalExactSimulator::step() {
  auto [bodies, bbox] = perform_exact_simulation_step(std::static_pointer_cast<ExactSimulationStep>(m_simulation_steps.back())->bodies(), m_dt, m_G);
  update_max_bbox(bbox);
  m_simulation_steps.push_back(std::make_shared<ExactSimulationStep>(std::move(bodies), std::move(bbox)));
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