#include "local_exact_simulator.h"

#include <algorithm>  // transform
#include <fstream>    // ofstream
#ifndef NDEBUG
#include <iomanip>  // setw
#include <iostream>
#endif
#include <execution>  // par_unseq
#include <memory>     // make_shared, static_pointer_cast
#include <utility>    // move

#include "body.h"
#include "body_update.h"
#include "bounding_box.h"
#include "exact_simulation_step.h"

namespace bh {

LocalExactSimulator::LocalExactSimulator(const std::string& filename, double dt, double G)
    : LocalExactSimulator(load(filename), dt, G) {}

LocalExactSimulator::LocalExactSimulator(std::vector<Body> step_zero, double dt, double G)
    : ISimulation(std::make_shared<ExactSimulationStep>(std::move(step_zero), compute_square_bounding_box(step_zero)), dt, G) {}

void LocalExactSimulator::step() {
  const auto& bodies = std::static_pointer_cast<ExactSimulationStep>(m_simulation_steps.back())->bodies();

#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  // This calls Body's default constructor m_n_bodies times, but we cannot do anything about it
  std::vector<Body> new_bodies(m_n_bodies);
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, bodies, m_dt, m_G);
                 });
#ifndef NDEBUG
  std::cout << "Computing new bounding box...\n";
#endif
  auto bbox = compute_square_bounding_box(bodies);

  m_simulation_steps.push_back(std::make_shared<ExactSimulationStep>(std::move(new_bodies), std::move(bbox)));
}

void LocalExactSimulator::save(const std::string& filename) const {
  json j = *this;
  std::ofstream o(filename);
#ifndef NDEBUG
  o << std::setw(2) << j;
#else
  o << to_json();
#endif
}

}  // namespace bh