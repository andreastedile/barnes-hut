#include "simulation.h"

#include <execution>
#include <fstream>  // ifstream
#include <iostream>
#include <numeric>    // transform_reduce
#include <stdexcept>  // runtime_error

#include "node.h"

namespace bh {

std::vector<Body> load(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  std::cout << "Reading file...\n";

  unsigned n_bodies;
  file >> n_bodies;

  std::vector<Body> bodies;
  bodies.reserve(n_bodies);
  for (unsigned i = 0; i < n_bodies; i++) {
    double position_x, position_y;
    double mass;
    double velocity_x, velocity_y;
    file >> position_x >> position_y;
    file >> mass;
    file >> velocity_x >> velocity_y;
    bodies.emplace_back(Vector2d{position_x, position_y}, mass,
                        Vector2d{velocity_x, velocity_y});
  }

  std::cout << "Read " << n_bodies << " bodies\n";
  return bodies;
}

AlignedBox2d compute_max_bbox(const std::vector<std::shared_ptr<SimulationStep>> &simulation_steps) {
  return std::transform_reduce(
      std::execution::par_unseq,
      simulation_steps.begin(), simulation_steps.end(),
      simulation_steps.front()->bbox(),
      [](const AlignedBox2d &max_so_far, const AlignedBox2d &curr) {
        return max_so_far.merged(curr);
      },
      [](const std::shared_ptr<SimulationStep> &step) {
        return step->bbox();
      });
}

ISimulation::ISimulation(std::shared_ptr<SimulationStep> step_zero, double dt, double G)
    : m_n_bodies{static_cast<int>(step_zero->bodies().size())},
      m_dt{dt},
      m_G(G),
      m_simulation_steps{std::move(step_zero)} {
#ifndef NDEBUG
  std::cout << "ISimulation constructor called\n";
#endif
}

const std::vector<std::shared_ptr<SimulationStep>> &ISimulation::steps() const {
  return m_simulation_steps;
}

void ISimulation::step_continuously(int n_steps) {
  for (int i = 0; i < n_steps; i++) {
    std::cout << "Step " << i << '\n';
    step();
  }
}

}  // namespace bh