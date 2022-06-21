#ifndef BARNES_HUT_LOCAL_SIMULATION_H
#define BARNES_HUT_LOCAL_SIMULATION_H

#include <vector>

#include "barnes_hut_simulation_step.h"
#include "simulation.h"

namespace bh {

class LocalBarnesHutSimulator final : public ISimulation {
 public:
  LocalBarnesHutSimulator(const std::string &filename, double dt, double G, double omega);
  void step() override;
  [[nodiscard]] json as_json() const override;
  void save_json() const override;
  [[nodiscard]] const std::vector<BarnesHutSimulationStep> &steps() const;

 private:
  std::vector<BarnesHutSimulationStep> m_steps;
};

void to_json(json &j, const LocalBarnesHutSimulator &simulator);

}  // namespace bh

#endif  // BARNES_HUT_LOCAL_SIMULATION_H
