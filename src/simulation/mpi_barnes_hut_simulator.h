#ifndef BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H

#include <eigen3/Eigen/Geometry>

using Eigen::AlignedBox2d;

namespace bh {

AlignedBox2d compute_bounding_box_for_processor(const AlignedBox2d& bbox, int n_procs, int proc_id);

}

#endif  // BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H
