#include <algorithm>
#ifndef NDEBUG
#include <iostream>
#endif

#include "barnes_hut/node.h"
#include "barnes_hut/simulation.h"

namespace bh {

void SimpleSimulator::step() {
  const SimulationStep& last_step = m_data.back();
  SimulationStep current_step;

  auto box = compute_minimum_bounding_box(last_step);
  Node quadtree(box.min(), box.max());

#ifndef NDEBUG
  std::cout << "Step " << m_curr_step
            << ". computed bounding box: bottom left (" << box.min().x() << " "
            << box.min().y() << "), top right: (" << box.min().x() << " "
            << box.max().y() << ")\n";
#endif

  std::for_each(last_step.begin(), last_step.end(),
                [&](const SimulatedBody& simulated) {
                  Body body(simulated.m_position, simulated.m_mass);
                  quadtree.insert(body);
                });

  std::transform(last_step.begin(), last_step.end(),
                 std::back_inserter(current_step),
                 [&](const SimulatedBody& body) {
                   return body.updated(quadtree, m_dt, m_force_algorithm_fn);
                 });

  m_data.push_back(current_step);
}

}  // namespace bh
