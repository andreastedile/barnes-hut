#include "mpi_barnes_hut_simulator.h"

#include <eigen3/Eigen/Eigen>

#include "bodies_gathering.h"
#include "body_update.h"   // update_body
#include "bounding_box.h"  // compute_square_bounding_box
#include "quadtree_gathering.h"
#ifndef NDEBUG
#include <cstdlib>  // puts
#endif
#include <algorithm>  // transform
#include <execution>  // par_unseq
#include <iterator>   // back_inserter
#include <utility>    // move

namespace bh {

MpiBarnesHutSimulator::MpiBarnesHutSimulator(double dt, double G, double theta, std::vector<Body> initial_bodies, int proc_id, int n_procs)
    : ISteppable(dt, {std::move(initial_bodies), compute_square_bounding_box(initial_bodies)}),
      IPhysics(G),
      IBarnesHut(theta),
      ICommunication(proc_id, n_procs) {}

BarnesHutSimulationStep MpiBarnesHutSimulator::step_impl(const BarnesHutSimulationStep& last_step) {
#ifndef NDEBUG
  if (proc_id == 0) {
    std::puts("Computing my bounding box...");
  }
#endif
  auto my_bbox = compute_bounding_box_for_processor(last_step.bbox(), proc_id, n_procs);

#ifndef NDEBUG
  if (proc_id == 0) {
    std::puts("Filtering bodies...");
  }
#endif
  const auto filtered_bodies = filter_bodies_by_subquadrant(last_step.bodies(), last_step.bbox(), my_bbox);

#ifndef NDEBUG
  std::puts("Constructing quadtree...");
#endif
  auto my_quadtree = construct_quadtree(my_bbox, filtered_bodies);

#ifndef NDEBUG
  std::puts("Gathering quadtree...");
#endif
  auto complete_quadtree = gather_quadtree(proc_id, n_procs, *my_quadtree);

#ifndef NDEBUG
  std::puts("Computing new bodies...");
#endif
  // If the number of processors does not evenly divide the number of bodies,
  // the processors are assigned different number of bodies to compute.
  // For example, with 6 bodies and 4 processors, the first 2 processors are assigned 2 bodies each,
  // and the other 2 processors are assigned 1 body each.
  // With 4 processors and 3 bodies, the first 3 processors are assigned 1 bodies each,
  // and the last processor is assigned none.
  const int total_n_bodies = static_cast<int>(last_step.bodies().size());
  const int n_remaining_bodies = total_n_bodies % n_procs;
  const int n_bodies_to_compute = (total_n_bodies / n_procs) + (proc_id < n_remaining_bodies);

  std::vector<Body> my_new_bodies(n_bodies_to_compute);
  const int idx_from = proc_id == 0 ? 0 : (proc_id - 1) * (total_n_bodies / n_procs) + std::min(proc_id - 1, n_remaining_bodies);
  std::transform(std::execution::par_unseq,
                 last_step.bodies().begin() + idx_from, last_step.bodies().begin() + idx_from + n_bodies_to_compute,
                 my_new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, *complete_quadtree, dt, G, theta);
                 });

  auto all_bodies = gather_bodies(proc_id, n_procs, total_n_bodies, my_new_bodies);

#ifndef NDEBUG
  if (proc_id == 0) {
    std::puts("Computing complete bounding box...");
  }
#endif
  const auto complete_bbox = compute_square_bounding_box(all_bodies);

  return {std::move(all_bodies), complete_bbox, std::move(complete_quadtree)};
}

std::vector<Body> filter_bodies_by_subquadrant(const std::vector<Body>& bodies, const Eigen::AlignedBox2d& outer_bbox, const Eigen::AlignedBox2d& own_bbox) {
  std::vector<Body> filtered;
  std::copy_if(bodies.begin(), bodies.end(), std::back_inserter(filtered), [&](const Body& body) {
    const bool body_at_right_of_own_bbox_left_side_inclusive = body.m_position.x() >= own_bbox.min().x();

    const bool body_at_left_of_own_bbox_right_side_exclusive = body.m_position.x() < own_bbox.max().x();
    const bool body_at_left_of_own_bbox_right_side_inclusive = body.m_position.x() <= own_bbox.max().x();

    const bool body_above_of_own_bbox_bottom_side_inclusive = body.m_position.y() >= own_bbox.min().y();

    const bool body_below_of_own_bbox_top_side_exclusive = body.m_position.y() < own_bbox.max().y();
    const bool body_below_of_own_bbox_top_side_inclusive = body.m_position.y() <= own_bbox.max().y();

    const bool own_bbox_top_side_lies_on_outer_bbox_top_side = own_bbox.max().y() == outer_bbox.max().y();

    const bool own_bbox_right_side_lies_on_outer_bbox_right_side = own_bbox.max().x() == outer_bbox.max().x();

    return body_at_right_of_own_bbox_left_side_inclusive &&
           ((body_at_left_of_own_bbox_right_side_exclusive && !own_bbox_right_side_lies_on_outer_bbox_right_side) ||
            (body_at_left_of_own_bbox_right_side_inclusive && own_bbox_right_side_lies_on_outer_bbox_right_side)) &&
           body_above_of_own_bbox_bottom_side_inclusive &&
           ((body_below_of_own_bbox_top_side_exclusive && !own_bbox_top_side_lies_on_outer_bbox_top_side) ||
            (body_below_of_own_bbox_top_side_inclusive && own_bbox_top_side_lies_on_outer_bbox_top_side));
  });
  return filtered;
}

Eigen::AlignedBox2d compute_bounding_box_for_processor(const Eigen::AlignedBox2d& outer_bbox, int proc_id, int n_procs) {
  const int N_ROWS = static_cast<int>(std::sqrt(n_procs));
  const int N_COLS = N_ROWS;

  double min_x = outer_bbox.min().x() + (proc_id % N_COLS) * (outer_bbox.sizes().x() / N_COLS);
  double max_x = min_x + (outer_bbox.sizes().x() / N_COLS);

  double min_y = outer_bbox.min().y() + std::floor(proc_id / N_ROWS) * (outer_bbox.sizes().y() / N_ROWS);
  double max_y = min_y + (outer_bbox.sizes().y() / N_ROWS);

  return {Eigen::Vector2d{min_x, min_y}, Eigen::Vector2d{max_x, max_y}};
}

}  // namespace bh
