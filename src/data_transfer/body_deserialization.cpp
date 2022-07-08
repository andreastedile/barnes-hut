#include "body_deserialization.h"


#ifdef WITH_TBB
#include <algorithm>
#include <execution>
#endif

namespace bh {

Body deserialize_body(const mpi::Body &body) {
  return {Eigen::Vector2d{body.position_x, body.position_y}, body.mass, Eigen::Vector2d{body.velocity_x, body.velocity_y}};
}

std::vector<Body> deserialize_bodies(const std::vector<mpi::Body> &bodies) {
  std::vector<Body> deserialized(bodies.size());
#ifdef WITH_TBB
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 deserialized.begin(),
                 [](const mpi::Body &body) {
                   return deserialize_body(body);
                 });
#else
#pragma omp parallel for
  for (size_t i = 0; i < bodies.size(); i++) {
    deserialized[i] = deserialize_body(bodies[i]);
  }
#endif
  return deserialized;
}

}  // namespace bh
