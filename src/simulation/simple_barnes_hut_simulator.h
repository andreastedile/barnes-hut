#ifndef BARNES_HUT_SIMPLE_BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_SIMPLE_BARNES_HUT_SIMULATOR_H

#include "simulation.h"

namespace bh {

class SimpleBarnesHutSimulator : public ISimulation {
  using ISimulation::ISimulation;
  void step() override;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMPLE_BARNES_HUT_SIMULATOR_H
