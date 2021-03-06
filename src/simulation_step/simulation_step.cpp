#include "simulation_step.h"
// Even though the #include below is reported unused, it allows serializing Eigen datatypes, and should not be removed!
#include <spdlog/spdlog.h>

#include <utility>  // move

#include "../eigen_json.h"

namespace bh {

SimulationStep::SimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox)
    : m_bodies(std::move(bodies)), m_bbox(bbox) {}

const std::vector<Body> &SimulationStep::bodies() const {
  return m_bodies;
}

const Eigen::AlignedBox2d &SimulationStep::bbox() const {
  return m_bbox;
}

#ifdef DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS

SimulationStep::SimulationStep(const SimulationStep &other)
    : m_bodies(other.m_bodies), m_bbox(other.m_bbox) {
  spdlog::trace("SimulationStep copy constructor");
}

SimulationStep::SimulationStep(SimulationStep &&other) noexcept
    : m_bodies(std::move(other.m_bodies)), m_bbox(other.m_bbox) {
  spdlog::trace("SimulationStep move constructor");
}

SimulationStep &SimulationStep::operator=(const SimulationStep &other) {
  spdlog::trace("SimulationStep copy assignment operator");
  m_bodies = other.m_bodies;
  m_bbox = other.m_bbox;
  return *this;
}

SimulationStep &SimulationStep::operator=(SimulationStep &&other) noexcept {
  spdlog::trace("SimulationStep move assignment operator");
  m_bodies = std::move(other.m_bodies);
  m_bbox = other.m_bbox;
  return *this;
}

#endif

}  // namespace bh