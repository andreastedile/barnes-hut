#include "body_serialization.h"

#ifdef WITH_TBB
#include <algorithm>
#include <execution>
#endif

namespace bh {

mpi::Body serialize_body(const Body& body) {
  return {body.m_position.x(), body.m_position.y(), body.m_velocity.x(), body.m_velocity.y(), body.m_mass};
}

std::vector<mpi::Body> serialize_bodies(const std::vector<Body>& bodies) {
  std::vector<mpi::Body> serialized(bodies.size());
#ifdef WITH_TBB
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 serialized.begin(),
                 [](const Body& body) {
                   return serialize_body(body);
                 });
#else
#pragma omp parallel for default(none) shared(bodies, serialized)
  for (size_t i = 0; i < bodies.size(); i++) {
    serialized[i] = serialize_body(bodies[i]);
  }
#endif
  return serialized;
}

}  // namespace bh