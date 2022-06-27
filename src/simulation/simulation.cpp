#include "simulation.h"

#include <fstream>  // ifstream
#include <iostream>
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

ISimulation::ISimulation(double dt, double G, double omega)
    : m_dt{dt}, m_G(G), m_omega(omega), m_max_bbox(AlignedBox2d{Vector2d{0, 0}, Vector2d{0, 0}}) {}

const std::vector<std::shared_ptr<SimulationStep>> &ISimulation::steps() const {
  return m_simulation_steps;
}

const AlignedBox2d &ISimulation::max_bbox() const {
  return m_max_bbox;
}

void ISimulation::step_continuously(int n_steps) {
  for (int i = 0; i < n_steps; i++) {
    std::cout << "Step " << i << '\n';
    step();
  }
}

void ISimulation::update_max_bbox(const AlignedBox2d& bbox) {
#ifndef NDEBUG
  if (bbox.min().x() < max_bbox().min().x()) {
    std::cout << "bounding box min x decreases from " << m_max_bbox.min().x() << " to " << bbox.min().x() << '\n';
  }
  if (bbox.min().y() < max_bbox().min().y()) {
    std::cout << "bounding box min y decreases from " << m_max_bbox.min().y() << " to " << bbox.min().y() << '\n';
  }
  if (bbox.max().x() > max_bbox().max().x()) {
    std::cout << "bounding box max x increases from " << m_max_bbox.max().x() << " to " << bbox.max().x() << '\n';
  }
  if (bbox.max().y() > max_bbox().max().y()) {
    std::cout << "bounding box max y increases from " << m_max_bbox.max().y() << " to " << bbox.max().y() << '\n';
  }
#endif

  m_max_bbox.min().x() = std::min(m_max_bbox.min().x(), bbox.min().x());
  m_max_bbox.min().y() = std::min(m_max_bbox.min().y(), bbox.min().y());
  m_max_bbox.max().x() = std::max(m_max_bbox.max().x(), bbox.max().x());
  m_max_bbox.max().y() = std::max(m_max_bbox.max().y(), bbox.max().y());
}

}  // namespace bh