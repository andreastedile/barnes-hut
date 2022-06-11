#ifndef BARNES_HUT_EXACT_SIMULATION_STEP_H
#define BARNES_HUT_EXACT_SIMULATION_STEP_H

#include "simulation_step.h"

namespace bh {

using ExactSimulationStep = SimulationStep;

void to_json(json &j, const ExactSimulationStep &step);

}

#endif  // BARNES_HUT_EXACT_SIMULATION_STEP_H
