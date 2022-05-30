#include "simulation.h"

#include <Eigen/Geometry>  // AlignedBox2d
#include <fstream>         // ifstream
#include <iostream>
#include <utility>  // move

#include "force.h"
#include "node.h"

using Eigen::AlignedBox2d;

namespace bh {

SimulatedBody::SimulatedBody(Vector2d position, double mass, Vector2d velocity)
    : Body(std::move(position), mass), m_velocity(std::move(velocity)) {}

SimulatedBody::SimulatedBody() : Body(), m_velocity(0, 0) {}

SimulationStep::SimulationStep(std::vector<SimulatedBody> bodies,
                               std::shared_ptr<const Node> quadtree)
    : m_bodies(std::move(bodies)), m_quadtree(std::move(quadtree)) {}

ISimulation::ISimulation(const std::string& filename, double dt,
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

  std::vector<SimulatedBody> bodies;
  for (unsigned i = 0; i < n_bodies; i++) {
    double position_x, position_y;
    double mass;
    double velocity_x, velocity_y;
    file >> position_x >> position_y;
    file >> mass;
    file >> velocity_x >> velocity_y;
    SimulatedBody body({position_x, position_y}, mass,
                       {velocity_x, velocity_y});
    bodies.push_back(std::move(body));
  }
  AlignedBox2d bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());
  SimulationStep step0(bodies, quadtree);
  m_data.push_back(std::move(step0));

  file.close();

  std::cout << "Read" << n_bodies << " bodies. Simulation is ready\n";
}

ISimulation::ISimulation(std::vector<SimulatedBody>&& bodies, double dt,
                         SimulationType type = APPROXIMATED)
    : m_dt(dt),
      m_force_algorithm_fn(type == APPROXIMATED
                               ? &compute_approximate_net_force_on_body
                               : &compute_exact_net_force_on_body) {
  AlignedBox2d bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());
  SimulationStep step0(bodies, quadtree);
  m_data.push_back(std::move(step0));
}

void ISimulation::run_continuously(unsigned n_steps) {
  for (unsigned i = 0; i < n_steps; i++) {
    m_curr_step++;
    step();
  }
}

SimulatedBody SimulatedBody::updated(
    const bh::Node& quadtree, double dt,
    const std::function<Vector2d(const Node&, const Body&)>&
        force_algorithm_fn = bh::compute_approximate_net_force_on_body) const {
  // The body's position is updated according to its current velocity
  Vector2d position(m_position + m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2d force = force_algorithm_fn(quadtree, *this);

  // The body's velocity is updated according to the net force on that particle.
  Vector2d velocity(m_velocity + force / m_mass * dt);

  return {position, m_mass, velocity};
}

}  // namespace bh
