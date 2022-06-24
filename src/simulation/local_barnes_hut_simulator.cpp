#include "local_barnes_hut_simulator.h"

#include <memory>   // make_shared
#include <utility>  // move

#include "barnes_hut_simulation_step.h"
#include "body.h"  // compute_square_bounding_box

namespace bh {

LocalBarnesHutSimulator::LocalBarnesHutSimulator(const std::string& filename, double dt, double G, double omega)
    : ISimulation(dt, G, omega) {
  std::vector<Body> bodies = load(filename);
  auto bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());
  auto simulation_step = std::make_shared<BarnesHutSimulationStep>(std::move(bodies), std::move(bbox), std::move(quadtree));
  m_simulation_steps.push_back(std::move(simulation_step));
}

void LocalBarnesHutSimulator::step() {
  auto [new_bodies, new_bbox, quadtree] = compute_new_bodies_barnes_hut(
      m_simulation_steps.back()->m_bodies, m_simulation_steps.back()->m_bbox, m_dt, m_G, m_omega);
  auto simulation_step = std::make_shared<BarnesHutSimulationStep>(std::move(new_bodies), std::move(new_bbox), std::move(quadtree));
  m_simulation_steps.push_back(std::move(simulation_step));
}

}  // namespace bh