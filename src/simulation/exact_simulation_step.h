#ifndef BARNES_HUT_EXACT_SIMULATION_STEP_H
#define BARNES_HUT_EXACT_SIMULATION_STEP_H

#include "simulation_step.h"

namespace bh {

class ExactSimulationStep final : public SimulationStep {
 public:
  ExactSimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d &bbox);
};

/**
 * @param dt simulation timestep; defines the accuracy of the computation: the  smaller, the more accurate
 * @param G gravitational constant
 */
ExactSimulationStep perform_exact_simulation_step(const ExactSimulationStep &simulation_step, double dt, double G);

void to_json(json &j, const ExactSimulationStep &step);

}  // namespace bh

#endif  // BARNES_HUT_EXACT_SIMULATION_STEP_H
