#ifndef BARNES_HUT_SIMULATION_EXACT_H
#define BARNES_HUT_SIMULATION_EXACT_H

#include <eigen3/Eigen/Eigen>
#include <memory>  // shared_ptr
#include <nlohmann/json.hpp>
#include <string>
#include <utility>  // tuple
#include <vector>

using json = nlohmann::json;
using Eigen::AlignedBox2d;

#include "body.h"
#include "node.h"
#include "simulation_step.h"

namespace bh {

/**
 * Loads some bodies from the specified file.
 * @todo describe the file format
 * @param filename file containing the bodies' data
 * @return bodies corresponding to the data in the file
 */
std::vector<Body> load(const std::string& filename);

class ISimulation {
 public:
  const int m_n_bodies;
  const double m_dt;
  const double m_G;

  ISimulation(std::shared_ptr<SimulationStep> step_zero, double dt, double G);

  /**
   * Performs a single simulation step
   */
  virtual void step() = 0;

  /**
   * @param n_steps number of simulation steps to perform
   */
  virtual void step_continuously(int n_steps) final;

  [[nodiscard]] virtual const std::vector<std::shared_ptr<SimulationStep>>& steps() const final;

  [[nodiscard]] virtual const AlignedBox2d& max_bbox() const final;

  /**
   * Saves the JSON representation of the simulation to a file
   * @todo maybe it's possible not to require subclasses to implement this,
   * but provide a default implementation ourselves
   */
  virtual void save(const std::string& filename) const = 0;

 protected:
  virtual void update_max_bbox(const AlignedBox2d& bbox) final;

  std::vector<std::shared_ptr<SimulationStep>> m_simulation_steps;
  AlignedBox2d m_max_bbox;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_EXACT_H
