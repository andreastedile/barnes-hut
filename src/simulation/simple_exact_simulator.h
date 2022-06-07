#ifndef BARNES_HUT_SIMPLE_EXACT_SIMULATOR_H
#define BARNES_HUT_SIMPLE_EXACT_SIMULATOR_H

#include "simulation.h"

namespace bh {

class SimpleExactSimulator : public ISimulation {
  using ISimulation::ISimulation;
  void step() override;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMPLE_EXACT_SIMULATOR_H
