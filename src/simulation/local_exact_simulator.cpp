#include "local_exact_simulator.h"

#include <utility>  // move

#include "body.h"  // compute_square_bounding_box

namespace bh {

LocalExactSimulator::LocalExactSimulator(const std::string& filename, double dt)
    : ISimulation<ExactSimulationStep>(dt) {
  auto bodies = load(filename);
  auto bbox = compute_square_bounding_box(bodies);
  m_steps.emplace_back(std::move(bodies), std::move(bbox));
}

void LocalExactSimulator::step() {
  auto new_bodies = compute_new_bodies_exact(m_steps.back().m_bodies, m_dt);
  auto bbox = compute_square_bounding_box(new_bodies);
  m_steps.emplace_back(std::move(new_bodies), std::move(bbox));
}

}  // namespace bh