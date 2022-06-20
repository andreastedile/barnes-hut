#include "local_exact_simulator.h"

#include <utility>  // move

#include "body.h"  // compute_square_bounding_box

namespace bh {

LocalExactSimulator::LocalExactSimulator(const std::string& filename, double dt)
    : ISimulation(dt) {
  auto bodies = load(filename);
  auto bbox = compute_square_bounding_box(bodies);
  m_steps.emplace_back(std::move(bodies), std::move(bbox));
}

void LocalExactSimulator::step() {
  auto [new_bodies, new_bbox] =
      compute_new_bodies_exact(m_steps.back().m_bodies, m_dt);
  m_steps.emplace_back(std::move(new_bodies), std::move(new_bbox));
}

const std::vector<ExactSimulationStep>& LocalExactSimulator::steps() const {
  return m_steps;
}

}  // namespace bh