#ifndef BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_SIMULATOR_H

#include <chrono>
#include <ostream>

#include "barnes_hut_simulation_step.h"

namespace bh {

struct Timings final {
  std::chrono::duration<double> construct_quadtree;
  std::chrono::duration<double> update_body;
  std::chrono::duration<double> compute_square_bounding_box;

  [[nodiscard]] std::chrono::duration<double> total() const;

  friend std::ostream& operator<<(std::ostream& os, const Timings& timings);
};

const Timings& timings();

BarnesHutSimulationStep step(const BarnesHutSimulationStep& last_step, double dt, double G, double theta);

}  // namespace bh

#endif  // BARNES_HUT_SIMULATOR_H
