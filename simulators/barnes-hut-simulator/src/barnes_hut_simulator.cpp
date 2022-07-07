#include "barnes_hut_simulator.h"

#include "body_update.h"   // update_body
#include "bounding_box.h"  // compute_square_bounding_box
#include "quadtree.h"      // construct_quadtree

#ifndef NDEBUG
#include <cstdlib>  // puts
#endif
#include <algorithm>  // transform
#include <execution>  // par_unseq
#include <utility>    // move

namespace bh {

BarnesHutSimulationStep step(const BarnesHutSimulationStep& last_step, double dt, double G, double theta) {
#ifndef NDEBUG
  std::puts("Constructing quadtree...");
#endif
  auto quadtree = construct_quadtree(last_step.bodies());

#ifndef NDEBUG
  std::puts("Computing new bodies...");
#endif
  std::vector<Body> new_bodies{last_step.bodies().size()};
  std::transform(std::execution::par_unseq,
                 last_step.bodies().begin(), last_step.bodies().end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, *quadtree, dt, G, theta);
                 });

#ifndef NDEBUG
  std::puts("Computing new bounding box...");
#endif
  auto new_bbox = compute_square_bounding_box(new_bodies);

  return {std::move(new_bodies), new_bbox, std::move(quadtree)};
}

}  // namespace bh
