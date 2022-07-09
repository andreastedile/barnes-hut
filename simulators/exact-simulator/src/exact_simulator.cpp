#include "exact_simulator.h"

#include "body_update.h"   // update_body
#include "bounding_box.h"  // compute_square_bounding_box

#ifndef NDEBUG
#include <cstdlib>  // puts
#endif
#ifdef WITH_TBB
#include <algorithm>  // transform
#include <execution>  // par_unseq
#endif
#include <utility>  // move

namespace bh {

SimulationStep step(const SimulationStep& last_step, double dt, double G) {
#ifndef NDEBUG
  std::puts("Computing new bodies...");
#endif
  std::vector<Body> new_bodies{last_step.bodies().size()};
#ifdef WITH_TBB
  std::transform(std::execution::par_unseq,
                 last_step.bodies().begin(), last_step.bodies().end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, last_step.bodies(), dt, G);
                 });
#else
#pragma omp parallel for default(none) shared(last_step, new_bodies, dt, G)
  for (size_t i = 0; i < last_step.bodies().size(); i++) {
    new_bodies[i] = update_body(last_step.bodies()[i], last_step.bodies(), dt, G);
  }
#endif

#ifndef NDEBUG
  std::puts("Computing new bounding box...");
#endif
  auto new_bbox = compute_square_bounding_box(new_bodies);

  return {std::move(new_bodies), new_bbox};
}

}  // namespace bh
