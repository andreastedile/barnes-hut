#include <iostream>
#include <string>

#include "local_barnes_hut_simulator.h"

int main(int, char* argv[]) {
  bh::LocalBarnesHutSimulator simulator(argv[1], std::stod(argv[2]));
  simulator.step_continuously(2);
  std::cout << "Simulation completed. Saving JSON...\n";
  simulator.save_json();
  return 0;
}
