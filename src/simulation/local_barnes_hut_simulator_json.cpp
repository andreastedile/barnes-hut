#include "barnes_hut_simulation_step.h"
#include "local_barnes_hut_simulator.h"
#include "../eigen_json.h"

namespace bh {

void to_json(json &j, const LocalBarnesHutSimulator &simulator) {
  json steps;
  std::transform(simulator.steps().begin(), simulator.steps().end(),
                 std::back_inserter(steps),
                 [](const auto &step) { return static_cast<const BarnesHutSimulationStep &>(*step); });

  j = json{{"dt",              simulator.m_dt},
           {"nSteps",          simulator.steps().size()},
           {"maxBoundingBox",  simulator.max_bbox()},
           {"simulationSteps", steps}};
}

}  // namespace bh
