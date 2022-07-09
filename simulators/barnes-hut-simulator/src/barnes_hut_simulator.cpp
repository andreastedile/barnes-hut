#include "barnes_hut_simulator.h"

#include <spdlog/spdlog.h>

#include "body_update.h"   // update_body
#include "bounding_box.h"  // compute_square_bounding_box
#include "quadtree.h"      // construct_quadtree

#ifdef WITH_TBB
#include <algorithm>  // transform
#include <execution>  // par_unseq
#endif
#include <utility>  // move

namespace bh {

BarnesHutSimulationStep step(const BarnesHutSimulationStep& last_step, double dt, double G, double theta) {
  spdlog::info("Constructing quadtree...");

  auto quadtree = construct_quadtree(last_step.bodies(), last_step.bbox());

#ifdef WITH_TBB
  spdlog::info("Computing new bodies (TBB)...");
#else
  spdlog::info("Computing new bodies (OpenMP)...");
#endif

  std::vector<Body> new_bodies{last_step.bodies().size()};
#ifdef WITH_TBB
  std::transform(std::execution::par_unseq,
                 last_step.bodies().begin(), last_step.bodies().end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, *quadtree, dt, G, theta);
                 });
#else
#pragma omp parallel for default(none) shared(last_step, new_bodies, quadtree, dt, G, theta)
  for (size_t i = 0; i < last_step.bodies().size(); i++) {
    new_bodies[i] = update_body(last_step.bodies()[i], *quadtree, dt, G, theta);
  }
#endif

  spdlog::info("Computing new bounding box...");

  auto new_bbox = compute_square_bounding_box(new_bodies);

  return {std::move(new_bodies), new_bbox, std::move(quadtree)};
}

}  // namespace bh
