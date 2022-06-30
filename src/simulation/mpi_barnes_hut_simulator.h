#ifndef BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H

#include <eigen3/Eigen/Geometry>
#include <memory>
#include <vector>

#include "mpi_datatypes.h"
#include "node.h"
#include "quadtree.h"

using Eigen::AlignedBox2d;

namespace bh {

AlignedBox2d compute_bounding_box_for_processor(const AlignedBox2d& bbox, int n_procs, int proc_id);

QuadtreeMatrix deserialize_branches(int n_procs, const std::vector<mpi::Node>& branches, const std::vector<int>& n_nodes);

}  // namespace bh

#endif  // BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H
