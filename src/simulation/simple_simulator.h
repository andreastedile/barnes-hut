#ifndef BARNES_HUT_SIMPLE_SIMULATOR_H
#define BARNES_HUT_SIMPLE_SIMULATOR_H

#include "simulation.h"

namespace bh {

class SimpleSimulator : public ISimulation {
  using ISimulation::ISimulation;
  void step() override;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMPLE_SIMULATOR_H
