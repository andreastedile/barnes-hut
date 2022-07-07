#ifndef BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_SIMULATOR_H

#include "barnes_hut_simulation_step.h"

namespace bh {

BarnesHutSimulationStep step(const BarnesHutSimulationStep& last_step, double dt, double G, double theta);

}  // namespace bh

#endif  // BARNES_HUT_SIMULATOR_H
