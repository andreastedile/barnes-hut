#include "../eigen_json.h"
#include "barnes_hut_simulation_step.h"
#include "mpi_barnes_hut_simulator.h"
#include "simulation.h"

namespace bh {

void to_json(json &j, const MpiBarnesHutSimulator &simulator) {
  json steps;
  std::transform(simulator.steps().begin(), simulator.steps().end(),
                 std::back_inserter(steps),
                 [](const auto &step) { return static_cast<const BarnesHutSimulationStep &>(*step); });

  j = json{{"dt", simulator.m_dt},
           {"nSteps", simulator.steps().size()},
           {"maxBoundingBox", compute_max_bbox(simulator.steps())},
           {"simulationSteps", steps}};
}

}  // namespace bh
