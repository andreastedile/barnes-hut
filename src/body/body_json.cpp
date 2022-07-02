#include "body.h"
// Do not remove the #include below! It allows serializing Eigen datatypes.
#include "../eigen_json.h"

namespace bh {

void to_json(nlohmann::json &j, const Body &body) {
  j = {{"position", body.m_position},
       {"mass", body.m_mass},
       {"velocity", body.m_velocity}};
}

}  // namespace bh