#include "simulation.h"

namespace bh {

void to_json(json &j, const SimulatedBody &body) {
  j = {{"position", {body.m_position.x(), body.m_position.y()}},
       {"velocity", {body.m_velocity.x(), body.m_velocity.y()}},
       {"mass", body.m_mass}};
}

void to_json(json &j, const SimulationStep &step) {
  j = {};
  std::transform(step.begin(), step.end(), std::back_inserter(j),
                 [](const SimulatedBody &body) { return body; });
}

void to_json(json &j, const SimulationData &data) {
  std::transform(data.begin(), data.end(), std::back_inserter(j),
                 [](const SimulationStep &step) { return step; });
}

void to_json(json &j, const ISimulation &simulation) {
  j = {{"dt", simulation.m_dt},
       {"n_steps", simulation.m_curr_step + 1},  // m_curr_step starts from 0
       {"data", simulation.m_data}};
}

}  // namespace bh
