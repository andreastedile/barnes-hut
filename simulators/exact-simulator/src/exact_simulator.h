#ifndef EXACT_SIMULATOR_H
#define EXACT_SIMULATOR_H

#include "simulation_step.h"

namespace bh {

SimulationStep step(const SimulationStep& last_step, double dt, double G);

}  // namespace bh

#endif  // EXACT_SIMULATOR_H
