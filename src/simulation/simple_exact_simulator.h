#ifndef BARNES_HUT_SIMPLE_EXACT_SIMULATOR_H
#define BARNES_HUT_SIMPLE_EXACT_SIMULATOR_H

#include "simulation.h"

namespace bh {

struct SimulationStep {
  std::vector<SimulatedBody> m_bodies;
  explicit SimulationStep(std::vector<SimulatedBody> bodies);
  // Copy constructor
  SimulationStep(const SimulationStep &other);
  // Move constructor
  SimulationStep(SimulationStep &&other) noexcept;
  // Copy assignment operator
  SimulationStep &operator=(const SimulationStep &other);
  // Move assignment operator
  SimulationStep &operator=(SimulationStep &&other) noexcept;
};

class SimpleExactSimulator final : public ISimulation {
 public:
  SimpleExactSimulator(const std::string &filename, double dt);
  SimpleExactSimulator(std::vector<SimulatedBody> bodies, double dt);

 private:
  void step() override;
  std::vector<SimulationStep> m_simulation_steps;
  friend void to_json(json &j, const SimpleExactSimulator &simulator);
  void save() const override;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMPLE_EXACT_SIMULATOR_H
