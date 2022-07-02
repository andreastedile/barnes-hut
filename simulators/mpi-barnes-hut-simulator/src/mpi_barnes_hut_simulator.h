#ifndef MPI_BARNES_HUT_SIMULATOR_H
#define MPI_BARNES_HUT_SIMULATOR_H

#include <eigen3/Eigen/Geometry>  // AlignedBox2d
#include <vector>

#include "barnes_hut.h"
#include "barnes_hut_simulation_step.h"
#include "body.h"
#include "communication.h"
#include "physics.h"
#include "quadtree.h"  // QuadtreeMatrix
#include "steppable.h"

namespace bh {

class MpiBarnesHutSimulator final : public ISteppable<BarnesHutSimulationStep>, public IPhysics, public IBarnesHut, public ICommunication {
 public:
  MpiBarnesHutSimulator(double dt, double G, double theta, std::vector<Body> initial_bodies, int proc_id, int n_procs);

 private:
  BarnesHutSimulationStep step_impl(const BarnesHutSimulationStep& last_step) override;
};

}  // namespace bh

#endif  // MPI_BARNES_HUT_SIMULATOR_H
