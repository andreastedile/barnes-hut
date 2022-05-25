#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <fstream>
#include <iostream>

#include "barnes_hut/node.h"

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
  return 0;
}
