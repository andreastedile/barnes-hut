#ifndef EXACT_SIMULATOR_H
#define EXACT_SIMULATOR_H

#include <vector>

#include "body.h"
#include "physics.h"
#include "simulation_step.h"
#include "steppable.h"

namespace bh {

class ExactSimulator final : public ISteppable<SimulationStep>, public IPhysics {
 public:
  ExactSimulator(double dt, double G, std::vector<Body> initial_bodies);

 private:
  SimulationStep step_impl(const SimulationStep& last_step) override;
};

}  // namespace bh

#endif  // EXACT_SIMULATOR_H
