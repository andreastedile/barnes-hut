#include "exact_simulator.h"

#include "body_update.h"   // update_body
#include "bounding_box.h"  // compute_square_bounding_box

#ifndef NDEBUG
#include <cstdlib>  // puts
#endif
#include <algorithm>  // transform
#include <execution>  // par_unseq
#include <utility>    // move

namespace bh {

ExactSimulator::ExactSimulator(double dt, double G, std::vector<Body> initial_bodies)
    : ISteppable(dt, {std::move(initial_bodies), compute_square_bounding_box(initial_bodies)}),
      IPhysics(G) {}

SimulationStep ExactSimulator::step_impl(const SimulationStep& last_step) {
#ifndef NDEBUG
  std::puts("Computing new bodies...");
#endif
  std::vector<Body> new_bodies{last_step.bodies().size()};
  std::transform(std::execution::par_unseq,
                 last_step.bodies().begin(), last_step.bodies().end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, last_step.bodies(), dt, G);
                 });

#ifndef NDEBUG
  std::puts("Computing new bounding box...");
#endif
  auto new_bbox = compute_square_bounding_box(new_bodies);

  return {std::move(new_bodies), new_bbox};
}

}  // namespace bh
