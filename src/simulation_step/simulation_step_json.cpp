#include <algorithm>
#include <algorithm>  // transform
#include <iterator>   // back_inserter

#include "simulation_step.h"

namespace bh {

void to_json(nlohmann::json &j, const SimulationStep &step) {
  nlohmann::json bodies;
  std::transform(step.bodies().begin(), step.bodies().end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j = nlohmann::json{{"bodies", bodies},
                     {"boundingBox", step.bodies()}};
}

}  // namespace bh