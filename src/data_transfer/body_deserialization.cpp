#include "body_deserialization.h"

#include <algorithm>
#include <execution>

namespace bh {

Body deserialize_body(const mpi::Body &body) {
  return {Eigen::Vector2d{body.position_x, body.position_y}, body.mass, Eigen::Vector2d{body.velocity_x, body.velocity_y}};
}

std::vector<Body> deserialize_bodies(const std::vector<mpi::Body> &bodies) {
  std::vector<Body> deserialized(bodies.size());
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 deserialized.begin(),
                 [](const mpi::Body &body) {
                   return deserialize_body(body);
                 });
  return deserialized;
}

}  // namespace bh
