#include "local_barnes_hut_simulator.h"

#include <fstream>  // ofstream
#ifndef NDEBUG
#include <iomanip>  // setw
#endif
#include <memory>   // make_shared, static_pointer_cast
#include <utility>  // move

#include "barnes_hut_simulation_step.h"
#include "body.h"  // compute_square_bounding_box

namespace bh {

LocalBarnesHutSimulator::LocalBarnesHutSimulator(const std::string& filename, double dt, double G, double omega)
    : ISimulation(dt, G, omega) {
  auto bodies = load(filename);
  m_max_bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(m_max_bbox.min(), m_max_bbox.max());
  m_simulation_steps.push_back(std::make_shared<BarnesHutSimulationStep>(std::move(bodies), m_max_bbox, std::move(quadtree)));
}

void LocalBarnesHutSimulator::step() {
  auto simulation_step = perform_barnes_hut_simulation_step(*std::static_pointer_cast<BarnesHutSimulationStep>(m_simulation_steps.back()), m_dt, m_G, m_omega);
  update_max_bbox(simulation_step.bbox());
  m_simulation_steps.push_back(std::make_shared<BarnesHutSimulationStep>(std::move(simulation_step)));
}

json LocalBarnesHutSimulator::to_json() const { return *this; }

void LocalBarnesHutSimulator::save() const {
  std::ofstream o("simulation.json");
#ifndef NDEBUG
  o << std::setw(2) << to_json();
#else
  o << to_json();
#endif
}

}  // namespace bh