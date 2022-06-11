#include <nlohmann/json.hpp>
using nlohmann::json;

#include "simulated_body.h"

namespace bh {

void to_json(json &j, const SimulatedBody &body) {
  // Todo: since that SimulatedBody extends Body, we can partially use Body's
  // own JSON serializer
  j = {{"position", {body.m_position.x(), body.m_position.y()}},
       {"mass", body.m_mass},
       {"velocity", {body.m_velocity.x(), body.m_velocity.y()}}};
}

}  // namespace bh
