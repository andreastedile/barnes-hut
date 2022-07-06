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

  j["quadtree"] = step.quadtree();
}

}  // namespace bh

#endif  // BARNES_HUT_BARNES_HUT_SIMULATION_STEP_JSON_H
