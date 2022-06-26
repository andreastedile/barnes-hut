#include "exact_simulation_step.h"
#include "local_exact_simulator.h"
#include "../eigen_json.h"

namespace bh {

void to_json(json &j, const LocalExactSimulator &simulator) {
  json steps;
  std::transform(simulator.steps().begin(), simulator.steps().end(),
                 std::back_inserter(steps),
                 [](const auto &step) { return static_cast<ExactSimulationStep &>(*step); });

  j = json{{"dt",              simulator.m_dt},
           {"nSteps",          simulator.steps().size()},
           {"maxBoundingBox",  simulator.max_bbox()},
           {"simulationSteps", steps}};
}

}  // namespace bh
