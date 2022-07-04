#include "mpi_barnes_hut_simulator.h"

#include <mpi.h>

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <execution>  // par_unseq
#include <fstream>    // ofstream
#include <iomanip>    // setw
#include <numeric>    // partial_sum
#include <utility>    // pair

#include "barnes_hut_simulation_step.h"
#ifndef NDEBUG
#include <iostream>
#endif
using Eigen::AlignedBox2d;
using Eigen::Vector2d;

#include "body.h"
#include "body_update.h"
#include "bounding_box.h"
#include "deserialization.h"
#include "serialization.h"

namespace bh {

std::vector<Body> filter_bodies_by_subquadrant(const std::vector<Body>& bodies, const AlignedBox2d& bbox, AlignedBox2d total_bbox) {
  std::vector<Body> filtered;
  std::copy_if(bodies.begin(), bodies.end(), std::back_inserter(filtered), [&bbox, &total_bbox](const Body& body) {
    return body.m_position.x() >= bbox.min().x() && ((body.m_position.x() < bbox.max().x() && bbox.max().x() < total_bbox.max().x()) || (body.m_position.x() <= bbox.max().x() && bbox.max().x() == total_bbox.max().x())) &&
           body.m_position.y() >= bbox.min().y() && ((body.m_position.y() < bbox.max().y() && bbox.max().y() < total_bbox.max().y()) || (body.m_position.y() <= bbox.max().y() && bbox.max().y() == total_bbox.max().y()));
  });
  return filtered;
}

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

std::pair<std::vector<mpi::Node>, std::vector<int>> gather_quadtree_branches(int n_procs, int proc_id, const std::vector<mpi::Node>& my_branch) {
  // contains the number of nodes that are to be received from each process
  std::vector<int> recv_n_nodes(n_procs);
  auto my_n_nodes = static_cast<int>(my_branch.size());
  MPI_Allgather(&my_n_nodes, 1, MPI_INT, &recv_n_nodes[0], 1, MPI_INT, MPI_COMM_WORLD);
  int total_n_nodes = std::accumulate(recv_n_nodes.begin(), recv_n_nodes.end(), 0, std::plus<>());

  // contains the number of bytes that are to be received from each process
  std::vector<int> recv_n_bytes(recv_n_nodes);
  std::transform(recv_n_nodes.begin(), recv_n_nodes.end(), recv_n_bytes.begin(), [](const auto n_nodes) -> int { return n_nodes * sizeof(mpi::Node); });

  // entry i specifies the displacement (relative to recvbuf) at which to place the incoming data from process i
  std::vector<int> displ(n_procs);
  std::partial_sum(recv_n_bytes.begin(), recv_n_bytes.end(), displ.begin() + 1, std::plus<>());

  std::vector<mpi::Node> branches(total_n_nodes);
  MPI_Allgatherv(&my_branch[0], recv_n_bytes[proc_id], MPI_BYTE, &branches[0], &recv_n_bytes[0], &displ[0], MPI_BYTE, MPI_COMM_WORLD);

  return {branches, recv_n_nodes};
}

std::vector<mpi::Body> gather_bodies(int n_procs, int proc_id,
                                     const std::vector<int>& recv_n_bodies,
                                     const std::vector<int>& recv_n_bytes,
                                     const std::vector<mpi::Body>& my_bodies) {
  int total_n_bodies = std::accumulate(recv_n_bodies.begin(), recv_n_bodies.end(), 0, std::plus<>());

  int total_n_bytes = std::accumulate(recv_n_bytes.begin(), recv_n_bytes.end(), 0, std::plus<>());

  // entry i specifies the displacement (relative to recvbuf) at which to place the incoming data from process i
  std::vector<int> displ(n_procs);
  std::partial_sum(recv_n_bytes.begin(), recv_n_bytes.end(), displ.begin() + 1, std::plus<>());

  std::vector<mpi::Body> branches(total_n_bodies);
  MPI_Allgatherv(&my_bodies[0], recv_n_bytes[proc_id], MPI_BYTE, &branches[0], &recv_n_bytes[0], &displ[0], MPI_BYTE, MPI_COMM_WORLD);

  return branches;
}

MpiBarnesHutSimulator MpiBarnesHutSimulator::from_file(int proc_id, int n_procs, const std::string& filename, double dt, double G, double omega) {
  auto initial_bodies = load(filename);
  int n_bodies = static_cast<int>(initial_bodies.size());
  auto bbox = compute_square_bounding_box(initial_bodies);

  std::vector<int> recv_n_bodies(n_procs, n_bodies / n_procs);
  std::transform(recv_n_bodies.begin(), recv_n_bodies.begin() + (n_bodies % n_procs), recv_n_bodies.begin(), [](const auto val) { return val + 1; });

  std::vector<int> recv_n_bytes(n_procs);
  std::transform(recv_n_bodies.begin(), recv_n_bodies.end(), recv_n_bytes.begin(), [](const auto n_nodes) { return n_nodes * sizeof(mpi::Body); });

  return {G, dt, omega,
          proc_id, n_procs,
          std::make_shared<BarnesHutSimulationStep>(initial_bodies, bbox),
          std::move(recv_n_bodies), std::move(recv_n_bytes),
          compute_bounding_box_for_processor(bbox, n_procs, proc_id)};
}

MpiBarnesHutSimulator::MpiBarnesHutSimulator(double G, double dt, double omega, int proc_id, int n_procs, std::shared_ptr<BarnesHutSimulationStep> step_zero, std::vector<int> recv_n_bodies, std::vector<int> recv_n_bytes, const AlignedBox2d& bbox)
    : ISimulation(std::move(step_zero), dt, G),
      m_omega{omega},
      m_proc_id{proc_id},
      m_n_procs{n_procs},
      m_recv_n_bodies(std::move(recv_n_bodies)),
      m_recv_n_bytes(std::move(recv_n_bytes)),
      m_bbox{bbox} {}

void MpiBarnesHutSimulator::step() {
#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Filtering bodies...\n";
  }
#endif
  const auto filtered_bodies = filter_bodies_by_subquadrant(m_simulation_steps.back()->bodies(), m_bbox, m_simulation_steps.back()->bbox());

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Constructing quadtree...\n";
  }
#endif
  auto my_quadtree = construct_quadtree(m_bbox, filtered_bodies);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Serializing quadtree...\n";
  }
#endif
  auto my_serialized_quadtree = serialize(*my_quadtree);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Gathering quadtree branches...\n";
  }
#endif
  auto [branches, n_nodes] = gather_quadtree_branches(m_n_procs, m_proc_id, my_serialized_quadtree);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Deserializing quadtree branches...\n";
  }
#endif
  auto matrix = deserialize_branches(m_n_procs, branches, n_nodes);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Reconstructing quadtree...\n";
  }
#endif
  auto complete_quadtree = reconstruct_quadtree(matrix);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Computing new bodies...\n";
  }
#endif
  int my_n_bodies = m_recv_n_bodies[m_proc_id];
  std::vector<Body> my_new_bodies(my_n_bodies);
  int idx_from = std::accumulate(m_recv_n_bodies.begin(), m_recv_n_bodies.begin() + m_proc_id, 0);
  std::transform(std::execution::par_unseq,
                 m_simulation_steps.back()->bodies().begin() + idx_from,
                 m_simulation_steps.back()->bodies().begin() + idx_from + my_n_bodies,
                 my_new_bodies.begin(),
                 [&](const Body& body) {
                   return update_body(body, *complete_quadtree, m_dt, m_G, m_omega);
                 });

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Serializing bodies...\n";
  }
#endif
  auto my_new_serialized_bodies = serialize(my_new_bodies);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Gathering bodies...\n";
  }
#endif
  auto all_serialized_bodies = gather_bodies(m_n_procs, m_proc_id, m_recv_n_bodies, m_recv_n_bytes, my_new_serialized_bodies);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Deserializing bodies...\n";
  }
#endif
  auto all_bodies = deserialize(all_serialized_bodies);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Computing complete bounding box...\n";
  }
#endif
  auto complete_bbox = compute_square_bounding_box(all_bodies);

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Updating maximum bounding box...\n";
  }
#endif

#ifndef NDEBUG
  if (m_proc_id == 0) {
    std::cout << "Computing bounding box...\n";
  }
#endif
  m_bbox = compute_bounding_box_for_processor(complete_bbox, m_n_procs, m_proc_id);

  auto simulation_step = std::make_shared<BarnesHutSimulationStep>(std::move(all_bodies), std::move(complete_bbox), std::move(complete_quadtree));
  if (m_proc_id == 0) {
    m_simulation_steps.push_back(std::move(simulation_step));
  } else {
    m_simulation_steps.back() = std::move(simulation_step);
  }
}


void MpiBarnesHutSimulator::save(const std::string& filename) const {
  if (m_proc_id == 0) {
    std::ofstream o(filename);
#ifndef NDEBUG
    json j = *this;
    o << std::setw(2) << j;
#else
    o << to_json();
#endif
  }
}

}  // namespace bh