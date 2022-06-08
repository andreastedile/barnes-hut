#include <algorithm>
#include <execution>
#include <utility>
#ifndef NDEBUG
#include <iostream>
#endif

#include "node.h"
#include "simple_barnes_hut_simulator.h"

namespace bh {

SimulationStep::SimulationStep(std::vector<SimulatedBody> bodies,
                               std::shared_ptr<const Node> quadtree)
    : m_bodies(std::move(bodies)), m_quadtree(std::move(quadtree)) {}

SimulationStep::SimulationStep(std::vector<SimulatedBody> bodies)
    : m_bodies(std::move(bodies)) {}

// Copy constructor
#ifdef DEBUG_COPY_CONSTRUCTOR
SimulationStep::SimulationStep(const SimulationStep &other)
    : m_bodies(other.m_bodies), m_quadtree(other.m_quadtree) {
  std::cout << "SimulationStep copy constructor\n";
}
#else
SimulationStep::SimulationStep(const SimulationStep &other) = default;
#endif

// Move constructor
#ifdef DEBUG_MOVE_CONSTRUCTOR
SimulationStep::SimulationStep(SimulationStep &&other) noexcept
    : m_bodies(std::move(other.m_bodies)),
      m_quadtree(std::move(other.m_quadtree)) {
  std::cout << "SimulationStep move constructor\n";
}
#else
SimulationStep::SimulationStep(SimulationStep &&other) noexcept = default;
#endif

// Copy assignment operator
#ifdef DEBUG_COPY_ASSIGNMENT_OPERATOR
SimulationStep &SimulationStep::operator=(const SimulationStep &other) {
  std::cout << "SimulationStep copy assignment operator\n";
  m_bodies = other.m_bodies;
  m_quadtree = other.m_quadtree;
  return *this;
}
#else
SimulationStep &SimulationStep::operator=(const SimulationStep &other) =
    default;
#endif

// Move assignment operator
#ifdef DEBUG_MOVE_ASSIGNMENT_OPERATOR
SimulationStep &SimulationStep::operator=(SimulationStep &&other) noexcept {
  std::cout << "SimulationStep move assignment operator\n";
  m_bodies = std::move(other.m_bodies);
  m_quadtree = std::move(other.m_quadtree);
  return *this;
}
#else
SimulationStep &SimulationStep::operator=(SimulationStep &&other) noexcept =
    default;
#endif

SimpleBarnesHutSimulator::SimpleBarnesHutSimulator(const std::string &filename,
                                                   double dt)
    : ISimulation(dt),
      m_simulation_steps{SimulationStep{read_file(filename)}} {}

SimpleBarnesHutSimulator::SimpleBarnesHutSimulator(
    std::vector<SimulatedBody> bodies, double dt)
    : ISimulation(dt), m_simulation_steps{SimulationStep{std::move(bodies)}} {}

void SimpleBarnesHutSimulator::step() {
  const std::vector<SimulatedBody> &bodies = m_simulation_steps.back().m_bodies;

  AlignedBox2d bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());

#ifndef NDEBUG
  std::cout << "Computed bounding box: bottom left (" << bbox.min().x() << " "
            << bbox.min().y() << "), top right: (" << bbox.max().x() << " "
            << bbox.max().y() << ")\n";
#endif

  std::for_each(bodies.begin(), bodies.end(),
                [&](const SimulatedBody &simulated) {
                  Body body(simulated.m_position, simulated.m_mass);
                  quadtree->insert(body);
                });
  // This calls SimulatedBody's default constructor bodies.size() times, but we
  // cannot do anything about it
  std::vector<SimulatedBody> updated_bodies(bodies.size());
  std::transform(std::execution::par_unseq, bodies.begin(), bodies.end(),
                 updated_bodies.begin(), [&](const SimulatedBody &body) {
                   return body.updated(*quadtree, m_dt);
                 });

  m_simulation_steps.emplace_back(std::move(updated_bodies),
                                  std::move(quadtree));
}

}  // namespace bh
