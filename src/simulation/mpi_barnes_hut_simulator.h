#ifndef BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H
#define BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H

#include <eigen3/Eigen/Geometry>
#include <memory>  // shared_ptr
#include <vector>

#include "barnes_hut_simulation_step.h"
#include "node.h"
#include "quadtree.h"
#include "simulation.h"

using Eigen::AlignedBox2d;

namespace bh {

AlignedBox2d compute_bounding_box_for_processor(const AlignedBox2d& bbox, int n_procs, int proc_id);

class MpiBarnesHutSimulator final : public ISimulation {
 public:
  static MpiBarnesHutSimulator from_file(int proc_id, int n_procs, const std::string& filename, double dt, double G, double omega);

  MpiBarnesHutSimulator(double G, double dt, double omega,
                        int proc_id, int n_procs,
                        std::shared_ptr<BarnesHutSimulationStep> step_zero,
                        std::vector<int> recv_n_bodies, std::vector<int> recv_n_bytes,
                        const AlignedBox2d& bbox);
  void step() override;
  void save(const std::string& filename) const override;

  const double m_omega;

 private:
  // identifier of this processor in the MPI system
  const int m_proc_id;
  // number of processors in the MPI system; by assumption, a power of 4
  const int m_n_procs;
  const std::vector<int> m_recv_n_bodies;
  const std::vector<int> m_recv_n_bytes;
  AlignedBox2d m_bbox;
};

void to_json(json& j, const MpiBarnesHutSimulator& simulator);

}  // namespace bh

#endif  // BARNES_HUT_MPI_BARNES_HUT_SIMULATOR_H
