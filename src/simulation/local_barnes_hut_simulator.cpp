#include "local_barnes_hut_simulator.h"

#include <fstream>  // ofstream
#ifndef NDEBUG
#include <iomanip>  // setw
#include <iostream>
#endif
#include <execution>  // par_unseq
#include <memory>     // make_shared, static_pointer_cast
#include <utility>    // move

#include "barnes_hut_simulation_step.h"
#include "body.h"
#include "body_update.h"
#include "bounding_box.h"
#include "quadtree.h"

namespace bh {

LocalBarnesHutSimulator::LocalBarnesHutSimulator(const std::string& filename, double dt, double G, double omega)
    : LocalBarnesHutSimulator(load(filename), dt, G, omega) {}

LocalBarnesHutSimulator::LocalBarnesHutSimulator(std::vector<Body> step_zero, double dt, double G, double omega)
    : ISimulation(std::make_shared<BarnesHutSimulationStep>(std::move(step_zero), compute_square_bounding_box(step_zero)), dt, G), m_omega{omega} {}

void LocalBarnesHutSimulator::step() {
#ifndef NDEBUG
  std::cout << "Constructing quadtree...\n";
#endif
  const auto& bodies = std::static_pointer_cast<BarnesHutSimulationStep>(m_simulation_steps.back())->bodies();
  auto quadtree = construct_quadtree(bodies);

#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  std::vector<Body> new_bodies(m_n_bodies);
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, *quadtree, m_dt, m_G, m_omega);
                 });

#ifndef NDEBUG
  std::cout << "Computing new  bounding box...\n";
#endif
  auto bbox = compute_square_bounding_box(bodies);

  m_simulation_steps.push_back(std::make_shared<BarnesHutSimulationStep>(std::move(new_bodies), std::move(bbox), std::move(quadtree)));
}

void LocalBarnesHutSimulator::save(const std::string& filename) const {
  json j = *this;
  std::ofstream o(filename);
#ifndef NDEBUG
  o << std::setw(2) << j;
#else
  o << to_json();
#endif
}

}  // namespace bh