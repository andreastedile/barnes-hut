#include <nlohmann/json.hpp>
using nlohmann::json;

#include "exact_simulation_step.h"
// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"

namespace bh {

void to_json(json &j, const ExactSimulationStep &step) {
  json bodies;
  std::transform(step.m_bodies.begin(), step.m_bodies.end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j = json{{"bodies", bodies},
           {"boundingBox", step.m_bbox}};
}

}  // namespace bh
