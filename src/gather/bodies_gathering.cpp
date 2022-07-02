#include <mpi.h>

#include <algorithm>   // transform
#include <functional>  // plus
#include <numeric>     // partial_sum

#include "body_deserialization.h"
#include "body_serialization.h"
#include "mpi_datatypes.h"

namespace bh {

std::vector<Body> gather_bodies(int proc_id, int n_procs, int total_n_bodies, const std::vector<Body>& my_bodies) {
  // contains the number of bodies that are to be received from each process
  std::vector<int> recv_n_bodies(n_procs, total_n_bodies / n_procs);
  std::transform(recv_n_bodies.begin(), recv_n_bodies.begin() + (total_n_bodies % n_procs), recv_n_bodies.begin(), [](const auto val) { return val + 1; });

  // contains the number of bytes that are to be received from each process
  std::vector<int> recv_n_bytes(n_procs);
  std::transform(recv_n_bodies.begin(), recv_n_bodies.end(), recv_n_bytes.begin(), [](const auto n_nodes) { return n_nodes * sizeof(mpi::Body); });

  // entry i specifies the displacement (relative to all_serialized_bodies) at which to place the incoming data from process i
  std::vector<int> displacements(n_procs);
  std::partial_sum(recv_n_bytes.begin(), recv_n_bytes.end(), displacements.begin() + 1, std::plus<>());

  std::vector<mpi::Body> my_serialized_bodies = serialize_bodies(my_bodies);

  std::vector<mpi::Body> all_serialized_bodies(total_n_bodies);  // note: here we must resize, and not reserve

  MPI_Allgatherv(&my_serialized_bodies[0], recv_n_bytes[proc_id], MPI_BYTE, &all_serialized_bodies[0], &recv_n_bytes[0], &displacements[0], MPI_BYTE, MPI_COMM_WORLD);

  return deserialize_bodies(all_serialized_bodies);
}

}  // namespace bh
