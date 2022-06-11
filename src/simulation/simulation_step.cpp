#include "simulation_step.h"

#ifndef NDEBUG
#include <iostream>
#endif
#include <utility>  // move

namespace bh {

SimulationStep::SimulationStep(std::vector<SimulatedBody> bodies,
                               Eigen::AlignedBox2d bbox)
    : m_bodies(std::move(bodies)), m_bbox(std::move(bbox)) {}

// Copy constructor
#ifdef DEBUG_COPY_CONSTRUCTOR
SimulationStep::SimulationStep(const SimulationStep &other)
    : m_bodies(other.m_bodies), m_bbox(other.m_bbox) {
  std::cout << "SimulationStep copy constructor\n";
}
#else
SimulationStep::SimulationStep(const SimulationStep &other) = default;
#endif

// Move constructor
#ifdef DEBUG_MOVE_CONSTRUCTOR
SimulationStep::SimulationStep(SimulationStep &&other) noexcept
    : m_bodies(std::move(other.m_bodies)), m_bbox(std::move(other.m_bbox)) {
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
  m_bbox = other.m_bbox;
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
  m_bbox = std::move(other.m_bbox);
  return *this;
}
#else
SimulationStep &SimulationStep::operator=(SimulationStep &&other) noexcept =
    default;
#endif

}  // namespace bh