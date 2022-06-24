#ifndef BARNES_HUT_LOCAL_SIMULATION_H
#define BARNES_HUT_LOCAL_SIMULATION_H

#include "simulation.h"

namespace bh {

class LocalBarnesHutSimulator final : public ISimulation {
 public:
  LocalBarnesHutSimulator(const std::string &filename, double dt, double G, double omega);
  std::shared_ptr<SimulationStep> step() override;
  [[nodiscard]] json to_json() const override;
  void save() const override;
};

void to_json(json &j, const LocalBarnesHutSimulator &simulator);

}  // namespace bh

#endif  // BARNES_HUT_LOCAL_SIMULATION_H
