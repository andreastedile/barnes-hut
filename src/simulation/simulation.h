#ifndef BARNES_HUT_SIMULATION_EXACT_H
#define BARNES_HUT_SIMULATION_EXACT_H

#include <eigen3/Eigen/Eigen>
#include <memory>  // shared_ptr
#include <nlohmann/json.hpp>
#include <string>
#include <type_traits>  // enable_if
#include <utility>      // tuple
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
std::vector<Body> load(const std::string &filename);

/**
 * Computes the exact position and velocity of the bodies after a simulation
 * step.
 * @param bodies for which to compute the updated position and velocity
 * @param dt simulation timestep; defines the accuracy of the computation: the
 * smaller, the more accurate
 * @return new bodies, containing the updated position and velocity
 */
std::tuple<std::vector<Body>, AlignedBox2d> compute_new_bodies_exact(
    const std::vector<Body> &bodies, double dt, double G);

/**
 * Computes the approximated position and velocity of the bodies after a
 * simulation step, using the Barnes–Hut approximation algorithm.
 * @param bodies for which to compute the updated position and velocity
 * @param dt simulation timestep; defines the accuracy of the computation: the
 * smaller, the more accurate
 * @return new bodies, containing the updated position and velocity, and the
 * quadtree that has been computed as part of the approximation algorithm
 */
std::tuple<std::vector<Body>, AlignedBox2d, std::shared_ptr<const Node>>
compute_new_bodies_barnes_hut(const std::vector<Body> &bodies,
                              const AlignedBox2d &bbox,
                              double dt, double G, double omega);

class ISimulation {
 public:
  const double m_dt;
  const double m_G;
  const double m_omega;

  /**
   * @param dt simulation timestep; defines the accuracy of the computation: the
   * smaller, the more accurate
   */
  explicit ISimulation(double dt, double G, double omega);

  /**
   * Performs a single simulation step
   */
  virtual std::shared_ptr<SimulationStep> step() = 0;

  /**
   * @param n_steps number of simulation steps to perform
   */
  virtual void step_continuously(int n_steps) final;

  [[nodiscard]] virtual const std::vector<std::shared_ptr<SimulationStep>> &steps() const final;

  /**
   * @return the JSON representation of the simulation
   */
  [[nodiscard]] virtual json to_json() const = 0;

  /**
   * Saves the JSON representation of the simulation to a file
   * @todo pass the filename as argument
   * @todo maybe it's possible not to require subclasses to implement this,
   * but provide a default implementation ourselves
   */
  virtual void save() const = 0;

 protected:
  std::vector<std::shared_ptr<SimulationStep>> m_simulation_steps;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_EXACT_H
