#ifndef BARNES_HUT_SIMULATION_H
#define BARNES_HUT_SIMULATION_H

#include <functional>
#include <memory>  // shared_ptr
#include <string>

#include "body.h"
#include "node.h"
#include "force.h"
#include "simulated_body.h"

using Eigen::Vector2d;

namespace bh {

std::vector<SimulatedBody> read_file(const std::string &filename);

class ISimulation {
 public:
  explicit ISimulation(double dt);

  virtual ~ISimulation() = default;
  /**
   * Runs the simulation for the specified number of steps.
   */
  virtual void run_continuously(unsigned n_steps) final;
  /**
   * Saves the results of the simulation to a JSON file.
   */
  virtual void save() const = 0;
  /**
   * Performs a single simulation step.
   */
  virtual void step() = 0;

 protected:
  unsigned m_curr_step = 0;
  double m_dt;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_H
