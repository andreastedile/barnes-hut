#ifndef MPI_BARNES_HUT_SIMULATOR_H
#define MPI_BARNES_HUT_SIMULATOR_H

#include <Eigen/Geometry>  // AlignedBox2d
#include <vector>

#include "barnes_hut_simulation_step.h"

namespace bh {

BarnesHutSimulationStep step(const BarnesHutSimulationStep& last_step, double dt, double G, double theta, int proc_id, int n_procs);

std::vector<Body> filter_bodies_by_subquadrant(const std::vector<Body>& bodies, const Eigen::AlignedBox2d& outer_bbox, const Eigen::AlignedBox2d& own_bbox);

Eigen::AlignedBox2d compute_bounding_box_for_processor(const Eigen::AlignedBox2d& outer_bbox, int proc_id, int n_procs);

}  // namespace bh

#endif  // MPI_BARNES_HUT_SIMULATOR_H
