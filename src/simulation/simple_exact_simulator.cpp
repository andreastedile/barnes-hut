#include <algorithm>
#include <execution>
#ifndef NDEBUG
#include <iostream>
#endif

#include "simple_exact_simulator.h"

namespace bh {

void SimpleExactSimulator::step() {
#ifndef NDEBUG
  std::cout << "Step " << m_curr_step << "\n";
#endif

  const std::vector<SimulatedBody>& bodies = m_data.back().m_bodies;

  std::vector<SimulatedBody> updated_bodies;
  updated_bodies.reserve(bodies.size());
  std::transform(std::execution::par_unseq, bodies.begin(), bodies.end(),
                 updated_bodies.begin(), [&](const SimulatedBody& body) {
                   return body.updated(bodies, m_dt);
                 });

  m_data.emplace_back(std::move(updated_bodies), nullptr);
}

}  // namespace bh
