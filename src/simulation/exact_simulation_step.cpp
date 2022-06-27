#include "exact_simulation_step.h"

#include "body_update.h"

#ifndef NDEBUG
#include <iostream>
#endif
#include <algorithm>  // transform
#include <execution>  // par_unseq
#include <utility>    // move

namespace bh {

ExactSimulationStep::ExactSimulationStep(std::vector<Body> bodies, const Eigen::AlignedBox2d& bbox)
    : SimulationStep(std::move(bodies), bbox) {}

ExactSimulationStep perform_exact_simulation_step(const ExactSimulationStep& simulation_step, double dt, double G) {
#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  // This calls Body's default constructor bodies.size() times, but we cannot do anything about it
  std::vector<Body> bodies(simulation_step.bodies().size());
  std::transform(std::execution::par_unseq,
                 simulation_step.bodies().begin(), simulation_step.bodies().end(),
                 bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, bodies, dt, G);
                 });
#ifndef NDEBUG
  std::cout << "Computing new bounding box...\n";
#endif
  auto bbox = compute_square_bounding_box(bodies);

  return {std::move(bodies), bbox};
}

}  // namespace bh