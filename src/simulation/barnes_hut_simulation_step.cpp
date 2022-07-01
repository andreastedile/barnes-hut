#include "barnes_hut_simulation_step.h"

#include "body_update.h"

#ifndef NDEBUG
#include <iostream>
#endif
#include <utility>  // move

namespace bh {

BarnesHutSimulationStep::BarnesHutSimulationStep(std::vector<Body> bodies, const AlignedBox2d &bbox)
    : SimulationStep(std::move(bodies), bbox), m_quadtree(std::make_shared<Node>(Vector2d{0,0}, Vector2d{0,0})) {}

BarnesHutSimulationStep::BarnesHutSimulationStep(std::vector<Body> bodies, const AlignedBox2d &bbox, std::shared_ptr<const Node> quadtree)
    : SimulationStep(std::move(bodies), bbox), m_quadtree(std::move(quadtree)) {}

const Node &BarnesHutSimulationStep::quadtree() const {
  return *m_quadtree;
}

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS

BarnesHutSimulationStep::BarnesHutSimulationStep(const BarnesHutSimulationStep &other)
    : SimulationStep(other), m_quadtree(other.m_quadtree) {
  std::cout << "BarnesHutSimulationStep copy constructor\n";
}

BarnesHutSimulationStep::BarnesHutSimulationStep(BarnesHutSimulationStep &&other) noexcept
    : SimulationStep(std::move(other.m_bodies), other.m_bbox),
      m_quadtree(std::move(other.m_quadtree)) {
  std::cout << "BarnesHutSimulationStep move constructor\n";
}

BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(const BarnesHutSimulationStep &other) {  // NOLINT(bugprone-unhandled-self-assignment)
  SimulationStep::operator=(other);
  std::cout << "BarnesHutSimulationStep copy assignment operator\n";
  m_quadtree = other.m_quadtree;
  return *this;
}

BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(BarnesHutSimulationStep &&other) noexcept {
  SimulationStep::operator=(std::move(other));
  std::cout << "BarnesHutSimulationStep move assignment operator\n";
  m_quadtree = std::move(other.m_quadtree);  // NOLINT(bugprone-use-after-move)
  return *this;
}

#endif

}  // namespace bh