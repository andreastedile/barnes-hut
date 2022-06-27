#include "barnes_hut_simulation_step.h"

#include "body_update.h"
#include "quadtree.h"

#ifndef NDEBUG
#include <iostream>
#endif
#include <algorithm>  // transform
#include <execution>  // par_unseq
#include <utility>    // move

namespace bh {

BarnesHutSimulationStep::BarnesHutSimulationStep(std::vector<Body> bodies, const AlignedBox2d &bbox, std::shared_ptr<const Node> quadtree)
    : SimulationStep(std::move(bodies), bbox),
      m_quadtree(std::move(quadtree)) {}

const Node &BarnesHutSimulationStep::quadtree() const {
  return *m_quadtree;
}

std::tuple<std::vector<Body>, AlignedBox2d, std::unique_ptr<Node>> perform_barnes_hut_simulation_step(const std::vector<Body> &bodies, double dt, double G, double omega) {
#ifndef NDEBUG
  std::cout << "Constructing quadtree...\n";
#endif
  auto quadtree = construct_quadtree(bodies);

#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  std::vector<Body> new_bodies(bodies.size());
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 new_bodies.begin(),
                 [&](const Body &body) {
                   return update_body(body, *quadtree, dt, G, omega);
                 });

#ifndef NDEBUG
  std::cout << "Computing new  bounding box...\n";
#endif
  auto bbox = compute_square_bounding_box(bodies);

  return {new_bodies, std::move(bbox), std::move(quadtree)};
}

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS

BarnesHutSimulationStep::BarnesHutSimulationStep(const BarnesHutSimulationStep &other)
    : SimulationStep(other), m_quadtree(other.m_quadtree) {
  std::cout << "BarnesHutSimulationStep copy constructor\n";
}

BarnesHutSimulationStep::BarnesHutSimulationStep(BarnesHutSimulationStep &&other) noexcept
    : SimulationStep(std::move(other.m_bodies), other.m_bbox),
      m_quadtree(std::move(other.m_quadtree)) {
  std::cout << "BarnesHutSimulationStep move constructor\n";
}

BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(const BarnesHutSimulationStep &other) {  // NOLINT(bugprone-unhandled-self-assignment)
  SimulationStep::operator=(other);
  std::cout << "BarnesHutSimulationStep copy assignment operator\n";
  m_quadtree = other.m_quadtree;
  return *this;
}

BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(BarnesHutSimulationStep &&other) noexcept {
  SimulationStep::operator=(std::move(other));
  std::cout << "BarnesHutSimulationStep move assignment operator\n";
  m_quadtree = std::move(other.m_quadtree);  // NOLINT(bugprone-use-after-move)
  return *this;
}

#endif

}  // namespace bh