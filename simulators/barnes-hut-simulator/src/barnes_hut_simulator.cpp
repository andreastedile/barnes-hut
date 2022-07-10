#include "barnes_hut_simulator.h"

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "body_update.h"   // update_body
#include "bounding_box.h"  // compute_square_bounding_box
#include "quadtree.h"      // construct_quadtree

#ifdef WITH_TBB
#include <algorithm>  // transform
#include <execution>  // par_unseq
#endif
#include <utility>  // move

namespace bh {

Timings m_timings;

std::chrono::duration<double> Timings::total() const {
  return construct_quadtree +
         update_body +
         compute_square_bounding_box;
}

std::ostream& operator<<(std::ostream& os, const Timings& timings) {
  os << "Quadtree construction: " << timings.construct_quadtree.count() << " s\n";
  os << "Bodies update: " << timings.update_body.count() << " s\n";
  os << "Bounding box computation: " << timings.compute_square_bounding_box.count() << " s\n";
  os << "Total: " << timings.total().count() << " s";
  return os;
}

const Timings& timings() {
  return m_timings;
}

BarnesHutSimulationStep step(const BarnesHutSimulationStep& last_step, double dt, double G, double theta) {
  spdlog::stopwatch sw;

  spdlog::info("Constructing quadtree...");
  auto quadtree = construct_quadtree(last_step.bodies(), last_step.bbox());

  m_timings.construct_quadtree += sw.elapsed();
  sw.reset();

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
#ifdef DEBUG_OPENMP_BODY_UPDATE_FOR_LOOP
    spdlog::debug("Updating body {}", i);
#endif
    new_bodies[i] = update_body(last_step.bodies()[i], *quadtree, dt, G, theta);
  }
#endif

  m_timings.update_body += sw.elapsed();
  sw.reset();

  spdlog::info("Computing new bounding box...");
  auto new_bbox = compute_square_bounding_box(new_bodies);

  m_timings.compute_square_bounding_box += sw.elapsed();

  return {std::move(new_bodies), new_bbox, std::move(quadtree)};
}

}  // namespace bh
