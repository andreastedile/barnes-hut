#include <nlohmann/json.hpp>
using nlohmann::json;

#include "barnes_hut_simulation_step.h"
// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"

namespace bh {

void to_json(json &j, const BarnesHutSimulationStep &step) {
  json bodies;
  std::transform(step.bodies().begin(), step.bodies().end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j = json{{"quadtree", step.quadtree()},
           {"bodies", bodies},
           {"boundingBox", step.bbox()}};
}

}  // namespace bh