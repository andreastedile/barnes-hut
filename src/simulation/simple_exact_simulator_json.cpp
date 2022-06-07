#include <fstream>  // ofstream
#include <nlohmann/json.hpp>

#include "simple_exact_simulator.h"

using json = nlohmann::json;

namespace bh {

void to_json(json &j, const SimulatedBody &body) {
  j = {{"position", {body.m_position.x(), body.m_position.y()}},
       {"velocity", {body.m_velocity.x(), body.m_velocity.y()}},
       {"mass", body.m_mass}};
}

void to_json(json &j, const SimulationStep &step) {
  j = {};
  json bodies;
  std::transform(step.m_bodies.begin(), step.m_bodies.end(),
                 std::back_inserter(bodies),
                 [](const SimulatedBody &body) { return body; });
  j["bodies"] = bodies;
}

void to_json(json &j, const std::vector<SimulationStep> &simulation_steps) {
  std::transform(simulation_steps.begin(), simulation_steps.end(),
                 std::back_inserter(j),
                 [](const SimulationStep &step) { return step; });
}

void to_json(json &j, const SimpleExactSimulator &simulator) {
  j = {{"dt", simulator.m_dt},
       {"n_steps", simulator.m_curr_step + 1},  // m_curr_step starts from 0
       {"simulation_steps", simulator.m_simulation_steps}};
}

void SimpleExactSimulator::save() const {
  json j = *this;
  std::ofstream o("simulation.json");
#ifndef NDEBUG
  o << j.dump(2) << std::endl;
#else
  o << j << std::endl;
#endif
}

}  // namespace bh
