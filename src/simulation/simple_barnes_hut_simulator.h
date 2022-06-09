#ifndef BARNES_HUT_SIMPLE_BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_SIMPLE_BARNES_HUT_SIMULATOR_H

#include <vector>

#include "simulation.h"

namespace bh {

struct SimulationStep {
  std::vector<SimulatedBody> m_bodies;
  std::shared_ptr<const Node> m_quadtree;
  SimulationStep(std::vector<SimulatedBody> bodies,
                 std::shared_ptr<const Node> quadtree);
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

class SimpleBarnesHutSimulator final : public ISimulation {
 public:
  SimpleBarnesHutSimulator(const std::string &filename, double dt);
  SimpleBarnesHutSimulator(std::vector<SimulatedBody> bodies, double dt);
  void save() const override;

 private:
  void step() override;
  std::vector<SimulationStep> m_simulation_steps;
  friend void to_json(json &j, const SimpleBarnesHutSimulator &simulator);
};

}  // namespace bh

#endif  // BARNES_HUT_SIMPLE_BARNES_HUT_SIMULATOR_H
