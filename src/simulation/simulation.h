#ifndef BARNES_HUT_SIMULATION_EXACT_H
#define BARNES_HUT_SIMULATION_EXACT_H

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <type_traits>  // enable_if
#include <utility>      // tuple

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
    const std::vector<Body> &bodies, double dt);

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
                              const AlignedBox2d &bbox, double dt);

class ISimulation {
 public:
  const double m_dt;

  /**
   * @param dt simulation timestep; defines the accuracy of the computation: the
   * smaller, the more accurate
   */
  explicit ISimulation(double dt) : m_dt{dt} {}

  /**
   * Performs a single simulation step
   */
  virtual void step() = 0;

  /**
   * @param n_steps number of simulation steps to perform
   */
  virtual void step_continuously(unsigned n_steps) final {
    for (unsigned i = 0; i < n_steps; i++) {
      std::cout << "Step " << i << '\n';
      step();
    }
  }

  /**
   * @return the JSON representation of the simulation
   */
  [[nodiscard]] virtual json as_json() const = 0;

  /**
   * Saves the JSON representation of the simulation to a file
   * @todo pass the filename as argument
   */
  virtual void save_json() const = 0;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_EXACT_H
