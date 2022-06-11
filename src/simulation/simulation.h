#ifndef BARNES_HUT_SIMULATION_EXACT_H
#define BARNES_HUT_SIMULATION_EXACT_H

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <type_traits>  // enable_if
#include <vector>

using json = nlohmann::json;

#include "node.h"
#include "simulation_step.h"

namespace bh {

std::vector<SimulatedBody> load(const std::string &filename);

std::vector<SimulatedBody> compute_new_bodies_exact(
    const std::vector<SimulatedBody> &bodies, double dt);

std::pair<std::shared_ptr<const Node>, std::vector<SimulatedBody>>
compute_new_bodies_barnes_hut(const std::vector<SimulatedBody> &bodies,
                              double dt);

template <typename T, typename = typename std::enable_if<
                          std::is_base_of<SimulationStep, T>::value, T>::type>
class ISimulation {
 public:
  const double m_dt;

  explicit ISimulation(double dt) : m_dt{dt} {}
  virtual void step() = 0;

  virtual void step_continuously(unsigned n_steps) final {
    for (unsigned i = 0; i < n_steps; i++) {
      std::cout << "Step " << i << '\n';
      step();
    }
  }
  [[nodiscard]] virtual json as_json() const = 0;
  virtual void save_json() const = 0;
  virtual const std::vector<T> &steps() const final { return m_steps; }

 protected:
  std::vector<T> m_steps;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_EXACT_H
