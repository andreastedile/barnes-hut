#include "simulation_step.h"

#ifndef NDEBUG
#include <iostream>
#endif
#include <utility>  // move

namespace bh {

SimulationStep::SimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox)
    : m_bodies(std::move(bodies)), m_bbox(bbox) {}

const std::vector<Body> &SimulationStep::bodies() const {
  return m_bodies;
}

const AlignedBox2d &SimulationStep::bbox() const {
  return m_bbox;
}

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS

SimulationStep::SimulationStep(const SimulationStep &other)
    : m_bodies(other.m_bodies), m_bbox(other.m_bbox) {
  std::cout << "SimulationStep copy constructor\n";
}

SimulationStep::SimulationStep(SimulationStep &&other) noexcept
    : m_bodies(std::move(other.m_bodies)), m_bbox(other.m_bbox) {
  std::cout << "SimulationStep move constructor\n";
}

SimulationStep &SimulationStep::operator=(const SimulationStep &other) {
  std::cout << "SimulationStep copy assignment operator\n";
  m_bodies = other.m_bodies;
  m_bbox = other.m_bbox;
  return *this;
}

SimulationStep &SimulationStep::operator=(SimulationStep &&other) noexcept {
  std::cout << "SimulationStep move assignment operator\n";
  m_bodies = std::move(other.m_bodies);
  m_bbox = other.m_bbox;
  return *this;
}

#endif

}  // namespace bh