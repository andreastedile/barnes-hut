#include "simulation.h"

#include <Eigen/Geometry>  // AlignedBox2d
#include <fstream>         // ifstream
#include <iostream>
#include <utility>  // move
#include <vector>

#include "force.h"
#include "node.h"

using Eigen::AlignedBox2d;

namespace bh {

SimulatedBody::SimulatedBody(Vector2d position, double mass, Vector2d velocity)
    : Body(std::move(position), mass), m_velocity(std::move(velocity)) {}

// Copy constructor
#ifdef DEBUG_COPY_CONSTRUCTOR
SimulatedBody::SimulatedBody(const SimulatedBody &other)
    : Body(other), m_velocity(other.m_velocity) {
  std::cout << "SimulatedBody copy constructor\n";
}
#else
SimulatedBody::SimulatedBody(const SimulatedBody &other) = default;
#endif

// Move constructor
#ifdef DEBUG_MOVE_CONSTRUCTOR
SimulatedBody::SimulatedBody(SimulatedBody &&other) noexcept
    : Body(std::move(other)), m_velocity(std::move(other.m_velocity)) {
  std::cout << "SimulatedBody move constructor\n";
}
#else
SimulatedBody::SimulatedBody(SimulatedBody &&other) noexcept = default;
#endif

// Copy assignment operator
#ifdef DEBUG_COPY_ASSIGNMENT_OPERATOR
SimulatedBody &SimulatedBody::operator=(const SimulatedBody &other) {
  Body::operator=(other);
  std::cout << "SimulatedBody copy assignment operator\n";
  m_velocity = other.m_velocity;
  return *this;
}
#else
SimulatedBody &SimulatedBody::operator=(const SimulatedBody &other) = default;
#endif

// Move assignment operator
#ifdef DEBUG_MOVE_ASSIGNMENT_OPERATOR
SimulatedBody &SimulatedBody::operator=(SimulatedBody &&other) noexcept {
  Body::operator=(std::move(other));
  std::cout << "SimulatedBody move assignment operator\n";
  m_velocity = std::move(other.m_velocity);
  return *this;
}
#else
SimulatedBody &SimulatedBody::operator=(SimulatedBody &&other) noexcept =
    default;
#endif

SimulationStep::SimulationStep(std::vector<SimulatedBody> bodies,
                               std::shared_ptr<const Node> quadtree)
    : m_bodies(std::move(bodies)), m_quadtree(std::move(quadtree)) {}

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

ISimulation::ISimulation(const std::string &filename, double dt) : m_dt(dt) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  std::cout << "Reading simulation data...\n";

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
    SimulatedBody body({position_x, position_y}, mass,
                       {velocity_x, velocity_y});
    bodies.push_back(std::move(body));
  }

  file.close();

  m_data.emplace_back(std::move(bodies), nullptr);

  std::cout << "Read " << n_bodies << " bodies. Simulation is ready\n";
}

ISimulation::ISimulation(std::vector<SimulatedBody> &bodies, double dt)
    : m_dt(dt) {
  AlignedBox2d bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());
  m_data.emplace_back(std::move(bodies), std::move(quadtree));
}

void ISimulation::run_continuously(unsigned n_steps) {
  m_data.reserve(n_steps);

  for (unsigned i = 0; i < n_steps; i++, m_curr_step++) {
    step();
  }
}

SimulatedBody SimulatedBody::updated(const bh::Node &quadtree,
                                     double dt) const {
  // The body's position is updated according to its current velocity
  Vector2d position(m_position + m_velocity * dt);

  // The net force on the particle is computed by adding the individual forces
  // from all the other particles.
  Vector2d force = compute_approximate_net_force_on_body(quadtree, *this);

  // The body's velocity is updated according to the net force on that particle.
  Vector2d velocity(m_velocity + force / m_mass * dt);

  return {position, m_mass, velocity};
}

}  // namespace bh
