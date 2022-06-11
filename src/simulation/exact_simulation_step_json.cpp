#include <nlohmann/json.hpp>
using nlohmann::json;

#include "exact_simulation_step.h"

namespace bh {

void to_json(json &j, const ExactSimulationStep &step) {
  json bodies;
  std::transform(step.m_bodies.begin(), step.m_bodies.end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j = {{"bodies", bodies},
       {"bounding_box",
        {{"bottom_left", {step.m_bbox.min().x(), step.m_bbox.min().y()}},
         {"top_right", {step.m_bbox.max().x(), step.m_bbox.max().y()}}}}};
}

}  // namespace bh
