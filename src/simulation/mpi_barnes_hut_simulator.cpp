#include "mpi_barnes_hut_simulator.h"

#include <eigen3/Eigen/Eigen>
#include <execution>  // par_unseq
#include <numeric>    // partial_sum
using Eigen::Vector2d;

#include "deserialization.h"

namespace bh {

AlignedBox2d compute_bounding_box_for_processor(const AlignedBox2d& bbox, int n_procs, int proc_id) {
  const int N_ROWS = static_cast<int>(std::sqrt(n_procs));
  const int N_COLS = N_ROWS;

  double min_x = bbox.min().x() + (proc_id % N_COLS) * (bbox.sizes().x() / N_COLS);
  double max_x = min_x + (bbox.sizes().x() / N_COLS);

  double min_y = bbox.min().y() + std::floor(proc_id / N_ROWS) * (bbox.sizes().y() / N_ROWS);
  double max_y = min_y + (bbox.sizes().y() / N_ROWS);

  return {Vector2d{min_x, min_y}, Vector2d{max_x, max_y}};
}

QuadtreeMatrix deserialize_branches(int n_procs, const std::vector<mpi::Node>& branches, const std::vector<int>& n_nodes) {
  const int N_ROWS = static_cast<int>(std::sqrt(n_procs));
  const int N_COLS = N_ROWS;

  QuadtreeMatrix matrix(N_ROWS);
  for (int i = 0; i < N_ROWS; i++) {
    matrix[i].resize(N_COLS);
  }

  std::vector<int> displacements(n_procs);
  std::partial_sum(n_nodes.begin(), n_nodes.end(),
                   displacements.begin() + 1,  // first displacement will be 0
                   std::plus<>());

  for (int i = 0; i < N_ROWS; i++) {
    for (int j = 0; j < N_COLS; j++) {
      const int idx_from = displacements[i * N_COLS + j];
      const int n_nodes_in_branch = n_nodes[i * N_COLS + j];
      const std::vector<mpi::Node> branch{branches.begin() + idx_from, branches.begin() + idx_from + n_nodes_in_branch};
      matrix[i][j] = deserialize_quadtree(branch);
    }
  }

  return matrix;
}

}  // namespace bh