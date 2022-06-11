#include "local_barnes_hut_simulator.h"

#include <memory>   // make_shared
#include <utility>  // move

namespace bh {

LocalBarnesHutSimulator::LocalBarnesHutSimulator(const std::string& filename,
                                                 double dt)
    : ISimulation<BarnesHutSimulationStep>(dt) {
  std::vector<SimulatedBody> bodies = load(filename);
  auto bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());
  m_steps.emplace_back(std::move(quadtree), std::move(bodies), std::move(bbox));
}

void LocalBarnesHutSimulator::step() {
  auto [quadtree, new_bodies] =
      compute_new_bodies_barnes_hut(m_steps.back().m_bodies, m_dt);
  m_steps.emplace_back(std::move(quadtree), std::move(new_bodies),
                       quadtree->bbox());
}
}  // namespace bh