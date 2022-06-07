#ifndef BARNES_HUT_SIMULATION_H
#define BARNES_HUT_SIMULATION_H

#include <functional>
#include <memory>  // shared_ptr
#include <string>
#include <vector>

#include "body.h"
#include "node.h"

using Eigen::Vector2d;

namespace bh {

struct SimulatedBody : Body {
  Vector2d m_velocity;
  SimulatedBody(Vector2d position, double mass, Vector2d velocity);
  [[nodiscard]] SimulatedBody updated(const bh::Node &quadtree,
                                      double dt) const;

  // Copy constructor
  SimulatedBody(const SimulatedBody &other);
  // Move constructor
  SimulatedBody(SimulatedBody &&other) noexcept;
  // Copy assignment operator
  SimulatedBody &operator=(const SimulatedBody &other);
  // Move assignment operator
  SimulatedBody &operator=(SimulatedBody &&other) noexcept;
};

struct SimulationStep {
  std::vector<SimulatedBody> m_bodies;
  std::shared_ptr<const Node> m_quadtree;
  SimulationStep(std::vector<SimulatedBody> bodies,
                 std::shared_ptr<const Node> quadtree);
  // Copy constructor
  SimulationStep(const SimulationStep &other);
  // Move constructor
  SimulationStep(SimulationStep &&other) noexcept;
  // Copy assignment operator
  SimulationStep &operator=(const SimulationStep &other);
  // Move assignment operator
  SimulationStep &operator=(SimulationStep &&other) noexcept;
};

class ISimulation {
 public:
  const double m_dt;
  /**
   * Creates a simulation with the bodies provided by a file.
   * @param filename file providing the bodies to add in the simulation
   * @param dt simulation timestep; defines the accuracy of the simulation
   * @param type of simulation
   */
  ISimulation(const std::string &filename, double dt);
  /**
   * Creates a simulation with the provided bodies.
   * @param bodies to add in the simulation
   * @param dt simulation timestep; defines the accuracy of the simulation
   * @param type of simulation
   */
  ISimulation(std::vector<SimulatedBody> &bodies, double dt);
  virtual ~ISimulation() = default;
  /**
   * Runs the simulation for the specified number of steps.
   */
  void run_continuously(unsigned n_steps);
  /**
   * Saves the results of the simulation to a JSON file.
   */
  void save();
  /**
   * Performs a single simulation step.
   */
  virtual void step() = 0;

 protected:
  std::vector<SimulationStep> m_data;
  unsigned int m_curr_step = 0;

 private:
  friend void to_json(json &j, const ISimulation &simulation);
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_H
