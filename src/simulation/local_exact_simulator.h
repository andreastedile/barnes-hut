#ifndef BARNES_HUT_LOCAL_EXACT_SIMULATOR_H
#define BARNES_HUT_LOCAL_EXACT_SIMULATOR_H

#include "exact_simulation_step.h"
#include "simulation.h"

namespace bh {

class LocalExactSimulator final : public ISimulation<ExactSimulationStep> {
 public:
  LocalExactSimulator(const std::string &filename, double dt);
  void step() override;
  [[nodiscard]] json as_json() const override;
  void save_json() const override;
};

void to_json(json &j, const LocalExactSimulator &simulator);

}  // namespace bh

#endif  // BARNES_HUT_LOCAL_EXACT_SIMULATOR_H
