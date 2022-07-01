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
#include "quadtree.h"

namespace bh {

LocalBarnesHutSimulator::LocalBarnesHutSimulator(const std::string& filename, double dt, double G, double omega)
    : ISimulation(dt, G), m_omega(omega) {
  auto bodies = load(filename);
  m_max_bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(m_max_bbox.min(), m_max_bbox.max());
  m_simulation_steps.push_back(std::make_shared<BarnesHutSimulationStep>(std::move(bodies), m_max_bbox, std::move(quadtree)));
}

void LocalBarnesHutSimulator::step() {
#ifndef NDEBUG
  std::cout << "Constructing quadtree...\n";
#endif
  const auto& bodies = std::static_pointer_cast<BarnesHutSimulationStep>(m_simulation_steps.back())->bodies();
  auto quadtree = construct_quadtree(bodies);

#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  std::vector<Body> new_bodies(bodies.size());
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

  update_max_bbox(bbox);

  m_simulation_steps.push_back(std::make_shared<BarnesHutSimulationStep>(std::move(new_bodies), std::move(bbox), std::move(quadtree)));
}

json LocalBarnesHutSimulator::to_json() const { return *this; }

void LocalBarnesHutSimulator::save(const std::string& filename) const {
  std::ofstream o(filename);
#ifndef NDEBUG
  o << std::setw(2) << to_json();
#else
  o << to_json();
#endif
}

}  // namespace bh