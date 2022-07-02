#include "quadtree_gathering.h"

#include <mpi.h>

#include <algorithm>   // transform
#include <functional>  // plus
#include <numeric>     // accumulate, partial_sum

#include "mpi_datatypes.h"
#include "quadtree.h"                  // QuadtreeMatrix, reconstruct_quadtree
#include "quadtree_deserialization.h"  // deserialize_quadtrees
#include "quadtree_serialization.h"    // serialize_quadtree

namespace bh {

std::unique_ptr<Node> gather_quadtree(int proc_id, int n_procs, const Node& my_quadtree) {
  // contains the number of nodes that are to be received from each process
  std::vector<int> recv_n_nodes(n_procs);
  auto my_n_nodes = my_quadtree.n_nodes();
  MPI_Allgather(&my_n_nodes, 1, MPI_INT, &recv_n_nodes[0], 1, MPI_INT, MPI_COMM_WORLD);

  int total_n_nodes = std::accumulate(recv_n_nodes.begin(), recv_n_nodes.end(), 0, std::plus<>());

  // contains the number of bytes that are to be received from each process
  std::vector<int> recv_n_bytes(recv_n_nodes);
  std::transform(recv_n_nodes.begin(), recv_n_nodes.end(), recv_n_bytes.begin(), [](const auto n_nodes) { return n_nodes * sizeof(mpi::Node); });

  // entry i specifies the displacement (relative to recvbuf) at which to place the incoming data from process i
  std::vector<int> displacements(n_procs);
  std::partial_sum(recv_n_bytes.begin(), recv_n_bytes.end(), displacements.begin() + 1, std::plus<>());

  std::vector<mpi::Node> my_serialized_quadtree = serialize_quadtree(my_quadtree);

  std::vector<mpi::Node> all_serialized_quadtrees(total_n_nodes);

  MPI_Allgatherv(&my_serialized_quadtree[0], recv_n_bytes[proc_id], MPI_BYTE, &all_serialized_quadtrees[0], &recv_n_bytes[0], &displacements[0], MPI_BYTE, MPI_COMM_WORLD);

  auto grid = deserialize_quadtrees(all_serialized_quadtrees, recv_n_nodes);

  return reconstruct_quadtree(grid);
}

}  // namespace bh
