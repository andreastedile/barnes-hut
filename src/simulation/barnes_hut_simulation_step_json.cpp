#include <nlohmann/json.hpp>
using nlohmann::json;

#include "barnes_hut_simulation_step.h"

namespace bh {

void to_json(json &j, const BarnesHutSimulationStep &step) {
  json bodies;
  std::transform(step.m_bodies.begin(), step.m_bodies.end(),
                 std::back_inserter(bodies),
                 [](const auto &step) { return step; });

  j = {{"quadtree", *step.m_quadtree},
       {"bodies", bodies},
       {"bounding_box",
        {{"bottom_left", {step.m_bbox.min().x(), step.m_bbox.min().y()}},
         {"top_right", {step.m_bbox.max().x(), step.m_bbox.max().y()}}}}};
}

}  // namespace bh