#ifndef BARNES_HUT_LOCAL_EXACT_SIMULATOR_H
#define BARNES_HUT_LOCAL_EXACT_SIMULATOR_H

#include "simulation.h"

namespace bh {

class LocalExactSimulator final : public ISimulation {
 public:
  LocalExactSimulator(const std::string &filename, double dt, double G, double omega);
  std::shared_ptr<SimulationStep> step() override;
  [[nodiscard]] json as_json() const override;
  void save_json() const override;
};

void to_json(json &j, const LocalExactSimulator &simulator);

}  // namespace bh

#endif  // BARNES_HUT_LOCAL_EXACT_SIMULATOR_H
