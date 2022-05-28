#include "simulation.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

#include "force.h"
#include "node.h"

namespace bh {

ISimulation::ISimulation(const std::string& filename, float dt,
                         SimulationType type = APPROXIMATED)
    : m_dt(dt),
      m_force_algorithm_fn(type == APPROXIMATED
                               ? &compute_approximate_net_force_on_body
                               : &compute_exact_net_force_on_body) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  std::cout << "Reading simulation data...\n";

  unsigned n_bodies;
  file >> n_bodies;

  SimulationStep step0;
  for (unsigned i = 0; i < n_bodies; i++) {
    float position_x, position_y;
    float mass;
    float velocity_x, velocity_y;
    file >> position_x >> position_y;
    file >> mass;
    file >> velocity_x >> velocity_y;
    SimulatedBody body({position_x, position_y}, mass,
                       {velocity_x, velocity_y});
    step0.push_back(body);
  }
  m_data.push_back(step0);

  file.close();

  std::cout << "Read" << n_bodies << " bodies. Simulation is ready\n";
}

ISimulation::ISimulation(const std::vector<SimulatedBody>& bodies, float dt,
                         SimulationType type = APPROXIMATED)
    : m_dt(dt),
      m_force_algorithm_fn(type == APPROXIMATED
                               ? &compute_approximate_net_force_on_body
                               : &compute_exact_net_force_on_body) {
  m_data.push_back(bodies);
  std::cout << "Loaded " << bodies.size() << " bodies. Simulation is ready\n";
}

void ISimulation::run_continuously(unsigned n_steps) {
  for (unsigned i = 0; i < n_steps; i++) {
    m_curr_step++;
    step();
  }
}

void ISimulation::save() {
  json j = *this;
  std::cout << j.dump(2) << std::endl;
  std::ofstream o("simulation.json");
  o << j.dump(2) << std::endl;
}

SimulatedBody SimulatedBody::updated(
    const bh::Node& quadtree, float dt,
    const std::function<Vector2f(const Node&, const Body&)>&
        force_algorithm_fn = bh::compute_approximate_net_force_on_body) const {
  // The body's position is updated according to its current velocity
  Vector2f position(m_position + m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2f force = force_algorithm_fn(quadtree, *this);

  // The body's velocity is updated according to the net force on that particle.
  Vector2f velocity(m_velocity + force / m_mass * dt);

  return {position, m_mass, velocity};
}

}  // namespace bh