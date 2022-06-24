#include <nlohmann/json.hpp>
using nlohmann::json;

#include "barnes_hut_simulation_step.h"
// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"

namespace bh {

void to_json(json &j, const BarnesHutSimulationStep &step) {
  json bodies;
  std::transform(step.m_bodies.begin(), step.m_bodies.end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j = json{{"quadtree", *step.m_quadtree},
           {"bodies", bodies},
           {"boundingBox", step.m_bbox}};
}

}  // namespace bh