#include <algorithm>
#ifndef NDEBUG
#include <iostream>
#endif

#include "node.h"
#include "simulation.h"

namespace bh {

void SimpleSimulator::step() {
#ifndef NDEBUG
  std::cout << "Step " << m_curr_step << "\n";
#endif

  const std::vector<SimulatedBody>& bodies = m_data.back().m_bodies;

  AlignedBox2f bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());

#ifndef NDEBUG
  std::cout << "Computed bounding box: bottom left (" << bbox.min().x() << " "
            << bbox.min().y() << "), top right: (" << bbox.max().x() << " "
            << bbox.max().y() << ")\n";
#endif

  std::for_each(bodies.begin(), bodies.end(),
                [&](const SimulatedBody& simulated) {
                  Body body(simulated.m_position, simulated.m_mass);
                  quadtree->insert(body);
                });

  std::vector<SimulatedBody> updated_bodies;
  std::transform(bodies.begin(), bodies.end(),
                 std::back_inserter(updated_bodies),
                 [&](const SimulatedBody& body) {
                   return body.updated(*quadtree, m_dt, m_force_algorithm_fn);
                 });

  SimulationStep step(updated_bodies, quadtree);
  m_data.push_back(std::move(step));
}

}  // namespace bh
