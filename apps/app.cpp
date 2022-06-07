#include <string>

#include "simple_barnes_hut_simulator.h"

int main(int argc, char* argv[]) {
  bh::SimpleBarnesHutSimulator simulator(argv[1], std::stod(argv[2]));
  simulator.run_continuously(1);
  simulator.save();

  return 0;
}
