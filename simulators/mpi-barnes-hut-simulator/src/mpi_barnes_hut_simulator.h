#ifndef MPI_BARNES_HUT_SIMULATOR_H
#define MPI_BARNES_HUT_SIMULATOR_H

#include <Eigen/Geometry>  // AlignedBox2d
#include <chrono>
#include <vector>

#include "barnes_hut_simulation_step.h"

namespace bh {

struct Timings final {
  std::chrono::duration<double> compute_bounding_box_for_processor;
  std::chrono::duration<double> filter_bodies_by_subquadrant;
  std::chrono::duration<double> construct_quadtree;
  std::chrono::duration<double> gather_quadtree;
  std::chrono::duration<double> update_body;
  std::chrono::duration<double> gather_bodies;
  std::chrono::duration<double> compute_square_bounding_box;

  [[nodiscard]] std::chrono::duration<double> total() const;

  friend std::ostream& operator<<(std::ostream& os, const Timings& timings);
};

const Timings& timings();

BarnesHutSimulationStep step(const BarnesHutSimulationStep& last_step, double dt, double G, double theta, int proc_id, int n_procs);

std::vector<Body> filter_bodies_by_subquadrant(const std::vector<Body>& bodies, const Eigen::AlignedBox2d& outer_bbox, const Eigen::AlignedBox2d& own_bbox);

Eigen::AlignedBox2d compute_bounding_box_for_processor(const Eigen::AlignedBox2d& outer_bbox, int proc_id, int n_procs);

}  // namespace bh

#endif  // MPI_BARNES_HUT_SIMULATOR_H
