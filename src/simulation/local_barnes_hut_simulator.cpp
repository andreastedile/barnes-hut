#include "local_barnes_hut_simulator.h"

#include <memory>   // make_shared
#include <utility>  // move

namespace bh {

LocalBarnesHutSimulator::LocalBarnesHutSimulator(const std::string& filename,
                                                 double dt)
    : ISimulation(dt) {
  std::vector<Body> bodies = load(filename);
  auto bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());
  m_steps.emplace_back(std::move(bodies), std::move(bbox), std::move(quadtree));
}

void LocalBarnesHutSimulator::step() {
  auto [new_bodies, new_bbox, quadtree] = compute_new_bodies_barnes_hut(
      m_steps.back().m_bodies, m_steps.back().m_bbox, m_dt);
  m_steps.emplace_back(std::move(new_bodies), std::move(new_bbox),
                       std::move(quadtree));
}

const std::vector<BarnesHutSimulationStep>& LocalBarnesHutSimulator::steps()
    const {
  return m_steps;
}

}  // namespace bh