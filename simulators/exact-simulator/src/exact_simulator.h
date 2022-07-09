#ifndef EXACT_SIMULATOR_H
#define EXACT_SIMULATOR_H

#include <chrono>

#include "simulation_step.h"

namespace bh {

struct Timings final {
  std::chrono::duration<double> update_body;
  std::chrono::duration<double> compute_square_bounding_box;

  [[nodiscard]] std::chrono::duration<double> total() const;

  friend std::ostream& operator<<(std::ostream& os, const Timings& timings);
};

const Timings& timings();

SimulationStep step(const SimulationStep& last_step, double dt, double G);

std::chrono::duration<double> update_body_cumulative_time();
std::chrono::duration<double> compute_square_bounding_box_cumulative_time();

}  // namespace bh

#endif  // EXACT_SIMULATOR_H
