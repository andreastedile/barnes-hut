#include "exact_simulation_step.h"

#include "body_update.h"

#ifndef NDEBUG
#include <iostream>
#endif
#include <algorithm>  // transform
#include <execution>  // par_unseq
#include <utility>    // move

namespace bh {

ExactSimulationStep::ExactSimulationStep(std::vector<Body> bodies, const AlignedBox2d& bbox)
    : SimulationStep(std::move(bodies), bbox) {}

std::pair<std::vector<Body>, AlignedBox2d> perform_exact_simulation_step(const std::vector<Body>& bodies, double dt, double G) {
#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  // This calls Body's default constructor bodies.size() times, but we cannot do anything about it
  std::vector<Body> new_bodies(bodies.size());
  std::transform(std::execution::par_unseq,
                 bodies.begin(), bodies.end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, bodies, dt, G);
                 });
#ifndef NDEBUG
  std::cout << "Computing new bounding box...\n";
#endif
  auto bbox = compute_square_bounding_box(bodies);

  return {new_bodies, bbox};
}

}  // namespace bh