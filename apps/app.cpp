#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <fstream>
#include <iostream>

#include "node.h"
#include "simulation.h"

int main() {
  bh::Node node(Eigen::Vector2f(0, 0), Eigen::Vector2f(10, 10));
  node.insert({{2.5, 7.5}, 1});
  node.insert({{5.625, 9.375}, 0.25});
  node.insert({{6.875, 8.125}, 0.25});
  node.insert({{8.125, 6.875}, 0.25});
  node.insert({{9.375, 5.625}, 0.25});
  json j = node;
  std::cout << j.dump(2) << std::endl;
  std::ofstream o("quadtree.json");
  o << j.dump(2) << std::endl;

  std::vector<bh::SimulatedBody> bodies;
  bodies.push_back({{2.5, 7.5}, 1, {0, 0}});
  bodies.push_back({{5.625, 9.375}, 0.25, {0, 0}});
  bodies.push_back({{6.875, 8.125}, 0.25, {0, 0}});
  bodies.push_back({{8.125, 6.875}, 0.25, {0, 0}});
  bodies.push_back({{9.375, 5.625}, 0.25, {0, 0}});
  bh::SimpleSimulator simulator(bodies, 0.01, bh::APPROXIMATED);
  simulator.run_continuously(500);
  simulator.save();
  return 0;
}
