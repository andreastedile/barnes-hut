#ifndef BARNES_HUT_STEPPABLE_H
#define BARNES_HUT_STEPPABLE_H

#include <iostream>
#include <vector>

namespace bh {

template <typename StepType>
class ISteppable {
 public:
  // this way of constructing the simulation_steps field produces the following output:
  //
  // Read 5 bodies
  // SimulationStep move constructor
  // BarnesHutSimulationStep move constructor
  explicit ISteppable(double dt, StepType step_zero) : dt{dt} {
    simulation_steps.reserve(500);
    simulation_steps.push_back(std::move(step_zero));
  };

  // this way of constructing the simulation_steps field produces the following output (notice the additional copy):
  //
  // SimulationStep move constructor
  // BarnesHutSimulationStep move constructor
  // SimulationStep copy constructor
  // BarnesHutSimulationStep copy constructor
  //
  // I should investigate why
  // explicit ISteppable(double dt, StepType step_zero) : dt{dt}, simulation_steps{std::move(step_zero)} {};

  /**
   * Performs a single simulation step
   */
  virtual StepType& step() final {
    std::cout << "step " << simulation_steps.size() << '\n';
    return simulation_steps.template emplace_back(step_impl(simulation_steps.back()));
  };

  const double dt;

 private:
  virtual StepType step_impl(const StepType& last_step) = 0;

  std::vector<StepType> simulation_steps;
};

}  // namespace bh

#endif  // BARNES_HUT_STEPPABLE_H
