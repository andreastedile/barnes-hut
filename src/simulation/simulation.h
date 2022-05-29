#ifndef BARNES_HUT_SIMULATION_H
#define BARNES_HUT_SIMULATION_H

#include <functional>
#include <memory>  // shared_ptr
#include <string>
#include <vector>

#include "body.h"
#include "node.h"

using Eigen::Vector2f;

namespace bh {

struct SimulatedBody : Body {
  const Vector2f m_velocity;
  SimulatedBody(Vector2f position, float mass, Vector2f velocity);
  [[nodiscard]] SimulatedBody updated(
      const bh::Node& quadtree, float dt,
      const std::function<Vector2f(const Node&, const Body&)>&
          m_force_algorithm_fn) const;
};

/**
 * Type of simulation.
 */
enum SimulationType {
  /**
   * The computation of the net forces acting on a single body is exact.
   */
  EXACT,
  /**
   * The computation of the net forces acting on a single body uses the
   * Barnes–Hut approximation algorithm.
   */
  APPROXIMATED
};

struct SimulationStep {
  const std::vector<SimulatedBody> m_bodies;
  const std::shared_ptr<const Node> m_quadtree;
  SimulationStep(std::vector<SimulatedBody> bodies,
                 std::shared_ptr<const Node> quadtree);
};

class ISimulation {
 public:
  const float m_dt;
  /**
   * Creates a simulation with the bodies provided by a file.
   * @param filename file providing the bodies to add in the simulation
   * @param dt simulation timestep; defines the accuracy of the simulation
   * @param type of simulation
   */
  ISimulation(const std::string& filename, float dt, SimulationType type);
  /**
   * Creates a simulation with the provided bodies.
   * @param bodies to add in the simulation
   * @param dt simulation timestep; defines the accuracy of the simulation
   * @param type of simulation
   */
  ISimulation(std::vector<SimulatedBody>&& bodies, float dt,
              SimulationType type);
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
  const std::function<Vector2f(const Node&, const Body&)> m_force_algorithm_fn;
  std::vector<SimulationStep> m_data;
  unsigned int m_curr_step = 0;

 private:
  friend void to_json(json& j, const ISimulation& simulation);
};

class SimpleSimulator : public ISimulation {
  using ISimulation::ISimulation;
  void step() override;
};

}  // namespace bh

#endif  // BARNES_HUT_SIMULATION_H
