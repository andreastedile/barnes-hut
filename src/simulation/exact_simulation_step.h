#ifndef BARNES_HUT_EXACT_SIMULATION_STEP_H
#define BARNES_HUT_EXACT_SIMULATION_STEP_H

#include "simulation_step.h"

namespace bh {

class ExactSimulationStep final : public SimulationStep {
 public:
  ExactSimulationStep(std::vector<Body> bodies, const AlignedBox2d &bbox);
};

void to_json(json &j, const ExactSimulationStep &step);

}  // namespace bh

#endif  // BARNES_HUT_EXACT_SIMULATION_STEP_H
