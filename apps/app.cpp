#include <iostream>
#include <string>

#include "local_barnes_hut_simulator.h"

int main(int, char* argv[]) {
  // Todo: refactor using an argument parser
  bh::LocalBarnesHutSimulator simulator(
      argv[1], // bodies.txt file
      std::stod(argv[3]), // dt
      std::stod(argv[4]), // G
      std::stod(argv[5])); // omega
  simulator.step_continuously(std::stoi(argv[2]));
  std::cout << "Simulation completed. Saving JSON...\n";
  simulator.save_json();
  return 0;
}
