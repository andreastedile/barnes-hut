#include <nlohmann/json.hpp>
using nlohmann::json;

#include "exact_simulation_step.h"
// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"

namespace bh {

void to_json(json &j, const ExactSimulationStep &step) {
  json bodies;
  std::transform(step.bodies().begin(), step.bodies().end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j = json{{"bodies", bodies},
           {"boundingBox", step.bodies()}};
}

}  // namespace bh
