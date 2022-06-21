#include "body.h"

namespace bh {

void to_json(json &j, const Body &body) {
  j = {{"position", {body.m_position.x(), body.m_position.y()}},
       {"mass", body.m_mass},
       {"velocity", body.m_velocity}};
}

}  // namespace bh