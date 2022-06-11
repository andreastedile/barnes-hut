#ifndef BARNES_HUT_LOCAL_SIMULATION_H
#define BARNES_HUT_LOCAL_SIMULATION_H

#include "barnes_hut_simulation_step.h"
#include "simulation.h"

namespace bh {

class LocalBarnesHutSimulator final
    : public ISimulation<BarnesHutSimulationStep> {
 public:
  LocalBarnesHutSimulator(const std::string &filename, double dt);
  void step() override;
  [[nodiscard]] json as_json() const override;
  void save_json() const override;
};

void to_json(json &j, const LocalBarnesHutSimulator &simulator);

}  // namespace bh

#endif  // BARNES_HUT_LOCAL_SIMULATION_H
