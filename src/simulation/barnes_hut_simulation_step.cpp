#include "barnes_hut_simulation_step.h"

#ifndef NDEBUG
#include <iostream>
#endif
#include <utility>  // move

namespace bh {

BarnesHutSimulationStep::BarnesHutSimulationStep(
    std::shared_ptr<const Node> quadtree, std::vector<SimulatedBody> bodies,
    Eigen::AlignedBox2d bbox)
    : SimulationStep(std::move(bodies), std::move(bbox)),
      m_quadtree(std::move(quadtree)) {}

// Copy constructor
#ifdef DEBUG_COPY_CONSTRUCTOR
BarnesHutSimulationStep::BarnesHutSimulationStep(
    const BarnesHutSimulationStep &other)
    : SimulationStep(other), m_quadtree(other.m_quadtree) {
  std::cout << "BarnesHutSimulationStep copy constructor\n";
}
#else
BarnesHutSimulationStep::BarnesHutSimulationStep(
    const BarnesHutSimulationStep &other) = default;
#endif

// Move constructor
#ifdef DEBUG_MOVE_CONSTRUCTOR
BarnesHutSimulationStep::BarnesHutSimulationStep(
    BarnesHutSimulationStep &&other) noexcept
    : SimulationStep(std::move(other.m_bodies), std::move(other.m_bbox)),
      m_quadtree(std::move(other.m_quadtree)) {
  std::cout << "BarnesHutSimulationStep move constructor\n";
}
#else
BarnesHutSimulationStep::BarnesHutSimulationStep(
    BarnesHutSimulationStep &&other) noexcept = default;
#endif

// Copy assignment operator
#ifdef DEBUG_COPY_ASSIGNMENT_OPERATOR
BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(
    const BarnesHutSimulationStep &other) {
  SimulationStep::operator=(other);
  std::cout << "BarnesHutSimulationStep copy assignment operator\n";
  m_quadtree = other.m_quadtree;
  return *this;
}
#else
BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(
    const BarnesHutSimulationStep &other) = default;
#endif

// Move assignment operator
#ifdef DEBUG_MOVE_ASSIGNMENT_OPERATOR
BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(
    BarnesHutSimulationStep &&other) noexcept {
  SimulationStep::operator=(std::move(other));
  std::cout << "BarnesHutSimulationStep move assignment operator\n";
  m_quadtree = std::move(other.m_quadtree);  // NOLINT(bugprone-use-after-move)
  return *this;
}
#else
BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(
    BarnesHutSimulationStep &&other) noexcept = default;
#endif

}  // namespace bh