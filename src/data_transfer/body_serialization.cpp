#include "body_serialization.h"

#include <algorithm>
#include <execution>

namespace bh {

mpi::Body serialize_body(const Body& body) {
  return {body.m_position.x(), body.m_position.y(), body.m_velocity.x(), body.m_velocity.y(), body.m_mass};
}

std::vector<mpi::Body> serialize_bodies(const std::vector<Body>& bodies) {
  std::vector<mpi::Body> serialized(bodies.size());
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 serialized.begin(),
                 [](const Body& body) {
                   return serialize_body(body);
                 });
  return serialized;
}

}  // namespace bh