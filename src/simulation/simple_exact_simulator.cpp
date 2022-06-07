#include <algorithm>
#include <execution>
#ifndef NDEBUG
#include <iostream>
#endif

#include "simple_exact_simulator.h"

namespace bh {

SimulationStep::SimulationStep(std::vector<SimulatedBody> bodies)
    : m_bodies(std::move(bodies)) {}

// Copy constructor
#ifdef DEBUG_COPY_CONSTRUCTOR
SimulationStep::SimulationStep(const SimulationStep &other)
    : m_bodies(other.m_bodies) {
  std::cout << "SimulationStep copy constructor\n";
}
#else
SimulationStep::SimulationStep(const SimulationStep &other) = default;
#endif

// Move constructor
#ifdef DEBUG_MOVE_CONSTRUCTOR
SimulationStep::SimulationStep(SimulationStep &&other) noexcept
    : m_bodies(std::move(other.m_bodies)) {
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
  return *this;
}
#else
SimulationStep &SimulationStep::operator=(SimulationStep &&other) noexcept =
    default;
#endif

SimpleExactSimulator::SimpleExactSimulator(const std::string &filename, double dt) : ISimulation(dt) {
  m_simulation_steps.emplace_back(SimulationStep{read_file(filename)});
}

void SimpleExactSimulator::step() {
  const std::vector<SimulatedBody>& bodies = m_simulation_steps.back().m_bodies;

  std::vector<SimulatedBody> updated_bodies;
  updated_bodies.reserve(bodies.size());
  std::transform(std::execution::par_unseq, bodies.begin(), bodies.end(),
                 updated_bodies.begin(), [&](const SimulatedBody& body) {
                   return body.updated(bodies, m_dt);
                 });

  m_simulation_steps.emplace_back(std::move(updated_bodies));
}

}  // namespace bh
