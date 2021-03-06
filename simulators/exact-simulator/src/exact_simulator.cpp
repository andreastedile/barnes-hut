#include "exact_simulator.h"

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <algorithm>  // transform
#include <execution>  // par_unseq
#include <utility>    // move

#include "body_update.h"   // update_body
#include "bounding_box.h"  // compute_square_bounding_box

namespace bh {

Timings m_timings;

std::chrono::duration<double> Timings::total() const {
  return update_body +
         compute_square_bounding_box;
}

std::ostream& operator<<(std::ostream& os, const Timings& timings) {
  os << "Bodies update: " << timings.update_body.count() << " s\n";
  os << "Bounding box computation: " << timings.compute_square_bounding_box.count() << " s\n";
  os << "Total: " << timings.total().count() << " s\n";
  return os;
}

const Timings& timings() {
  return m_timings;
}

SimulationStep step(const SimulationStep& last_step, double dt, double G) {
  spdlog::stopwatch sw;

  std::vector<Body> new_bodies{last_step.bodies().size()};

#if defined(WITH_TBB) and defined(PARALLEL_BODIES_UPDATE)
  spdlog::debug("Computing new bodies in parallel (TBB) (update of each single body is serial)...");

  std::transform(std::execution::par_unseq,
                 last_step.bodies().begin(), last_step.bodies().end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body_serial(body, last_step.bodies(), dt, G);
                 });

#elif defined(WITH_TBB) and not defined(PARALLEL_BODIES_UPDATE)
  spdlog::debug("Computing new bodies serially (update of each single body is parallel (TBB))...");

  std::transform(last_step.bodies().begin(), last_step.bodies().end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body_parallel(body, last_step.bodies(), dt, G);
                 });

#elif not defined(WITH_TBB) and defined(PARALLEL_BODIES_UPDATE)
  spdlog::debug("Computing new bodies in parallel (OpenMP) (update of each single body is serial)...");

#pragma omp parallel for default(none) shared(last_step, new_bodies, dt, G)
  for (size_t i = 0; i < last_step.bodies().size(); i++) {
#ifdef DEBUG_OPENMP_BODY_UPDATE_FOR_LOOP
    spdlog::trace("Updating body {}", i);
#endif
    new_bodies[i] = update_body_serial(last_step.bodies()[i], last_step.bodies(), dt, G);
  }

#elif not defined(WITH_TBB) and not defined(PARALLEL_BODIES_UPDATE)
  spdlog::debug("Computing new bodies serially (update of each single body is parallel (OpenMP))...");

  std::transform(last_step.bodies().begin(), last_step.bodies().end(),
                 new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body_parallel(body, last_step.bodies(), dt, G);
                 });
#endif

  m_timings.update_body += sw.elapsed();
  sw.reset();

  spdlog::debug("Computing new bounding box...");
  auto new_bbox = compute_square_bounding_box(new_bodies);

  m_timings.compute_square_bounding_box += sw.elapsed();

  return {std::move(new_bodies), new_bbox};
}

}  // namespace bh
