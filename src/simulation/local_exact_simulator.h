#ifndef BARNES_HUT_LOCAL_EXACT_SIMULATOR_H
#define BARNES_HUT_LOCAL_EXACT_SIMULATOR_H

#include <vector>

#include "exact_simulation_step.h"
#include "simulation.h"

namespace bh {

class LocalExactSimulator final : public ISimulation {
 public:
  LocalExactSimulator(const std::string &filename, double dt);
  void step() override;
  [[nodiscard]] json as_json() const override;
  void save_json() const override;
  [[nodiscard]] const std::vector<ExactSimulationStep> &steps() const;

 private:
  std::vector<ExactSimulationStep> m_steps;
};

void to_json(json &j, const LocalExactSimulator &simulator);

}  // namespace bh

#endif  // BARNES_HUT_LOCAL_EXACT_SIMULATOR_H
