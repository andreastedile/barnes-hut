#ifndef BARNES_HUT_BARNES_HUT_SIMULATION_STEP_JSON_H
#define BARNES_HUT_BARNES_HUT_SIMULATION_STEP_JSON_H

#include <algorithm>  // transform
#include <iterator>   // back_inserter

#include "barnes_hut_simulation_step.h"
// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"

namespace bh {

void to_json(nlohmann::json &j, const BarnesHutSimulationStep &step) {
  j = static_cast<const SimulationStep &>(step);

  nlohmann::json bodies;
  std::transform(step.bodies().begin(), step.bodies().end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j["boundingBox"] = step.bbox();
}

}  // namespace bh

#endif  // BARNES_HUT_BARNES_HUT_SIMULATION_STEP_JSON_H
