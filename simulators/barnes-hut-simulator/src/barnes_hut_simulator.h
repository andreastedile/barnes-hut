#ifndef BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_SIMULATOR_H

#include <vector>

#include "barnes_hut.h"
#include "barnes_hut_simulation_step.h"
#include "body.h"
#include "physics.h"
#include "steppable.h"

namespace bh {

class BarnesHutSimulator final : public ISteppable<BarnesHutSimulationStep>, public IPhysics, public IBarnesHut {
 public:
  BarnesHutSimulator(double dt, double G, double theta, std::vector<Body> initial_bodies);

 private:
  BarnesHutSimulationStep step_impl(const BarnesHutSimulationStep& last_step) override;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATOR_H
