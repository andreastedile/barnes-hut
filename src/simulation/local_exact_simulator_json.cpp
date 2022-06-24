#include <fstream>  // ofstream

#include "exact_simulation_step.h"
#include "local_exact_simulator.h"

namespace bh {

void to_json(json &j, const LocalExactSimulator &simulator) {
  json steps;
  std::transform(simulator.steps().begin(), simulator.steps().end(),
                 std::back_inserter(steps),
                 [](const auto &step) { return static_cast<ExactSimulationStep &>(*step); });

  j = {{"dt", simulator.m_dt},
       {"n_steps", simulator.steps().size()},
       {"simulation_steps", steps}};
}

json LocalExactSimulator::as_json() const { return *this; }

void LocalExactSimulator::save_json() const {
  json j = as_json();
  std::ofstream o("simulation.json");
#ifndef NDEBUG
  o << j.dump(2) << std::endl;
#else
  o << j << std::endl;
#endif
}

}  // namespace bh
