#include "mpi_barnes_hut_simulator.h"

#include <eigen3/Eigen/Eigen>
using Eigen::Vector2d;

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

}  // namespace bh