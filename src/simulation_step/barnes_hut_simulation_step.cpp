#include "barnes_hut_simulation_step.h"
// Even though the #include below is reported unused, it allows serializing Eigen datatypes, and should not be removed!
#include "../eigen_json.h"

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS
#include <spdlog/spdlog.h>
#endif
#include <utility>    // move
#include <memory>

namespace bh {

BarnesHutSimulationStep::BarnesHutSimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox)
    : SimulationStep(std::move(bodies), bbox), m_quadtree(std::make_unique<Node>(Eigen::Vector2d{0, 0}, Eigen::Vector2d{0, 0})) {}

BarnesHutSimulationStep::BarnesHutSimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox, std::shared_ptr<const Node> quadtree)
    : SimulationStep(std::move(bodies), bbox), m_quadtree(std::move(quadtree)) {}

const Node &BarnesHutSimulationStep::quadtree() const {
  return *m_quadtree;
}

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS

BarnesHutSimulationStep::BarnesHutSimulationStep(const BarnesHutSimulationStep &other)
    : SimulationStep(other), m_quadtree(other.m_quadtree) {
  spdlog::trace("BarnesHutSimulationStep copy constructor");
}

BarnesHutSimulationStep::BarnesHutSimulationStep(BarnesHutSimulationStep &&other) noexcept
    : SimulationStep(std::move(*this)),
      m_quadtree(std::move(other.m_quadtree)) {
  spdlog::trace("BarnesHutSimulationStep move constructor");
}

BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(const BarnesHutSimulationStep &other) {  // NOLINT(bugprone-unhandled-self-assignment)
  SimulationStep::operator=(other);
  spdlog::trace("BarnesHutSimulationStep copy assignment operator");
  m_quadtree = other.m_quadtree;
  return *this;
}

BarnesHutSimulationStep &BarnesHutSimulationStep::operator=(BarnesHutSimulationStep &&other) noexcept {
  SimulationStep::operator=(std::move(other));
  spdlog::trace("BarnesHutSimulationStep move assignment operator");
  m_quadtree = std::move(other.m_quadtree);  // NOLINT(bugprone-use-after-move)
  return *this;
}

#endif

}  // namespace bh