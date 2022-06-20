#include <iostream>
#include <string>

#include "local_barnes_hut_simulator.h"

int main(int, char* argv[]) {
  bh::LocalBarnesHutSimulator simulator(argv[1], std::stod(argv[3]));
  simulator.step_continuously(std::stoi(argv[2]));
  std::cout << "Simulation completed. Saving JSON...\n";
  simulator.save_json();
  return 0;
}
