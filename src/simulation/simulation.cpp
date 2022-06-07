#include "simulation.h"

#include <Eigen/Geometry>  // AlignedBox2d
#include <fstream>         // ifstream
#include <iostream>
#include <memory>   // make_shared
#include <vector>

#include "node.h"

using Eigen::AlignedBox2d;

namespace bh {

std::vector<SimulatedBody> read_file(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  std::cout << "Reading file...\n";

  unsigned n_bodies;
  file >> n_bodies;

  std::vector<SimulatedBody> bodies;
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

  std::cout << "Read " << bodies.size() << " bodies\n";
  return bodies;
}

ISimulation::ISimulation(double dt) : m_dt(dt) {}

void ISimulation::run_continuously(unsigned n_steps) {
  for (unsigned i = 0; i < n_steps; i++, m_curr_step++) {
    std::cout << "step " << m_curr_step << "\n";
    step();
  }
}

}  // namespace bh
