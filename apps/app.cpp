#include <string>

#include "local_barnes_hut_simulator.h"

int main(int argc, char* argv[]) {
  bh::LocalBarnesHutSimulator simulator(argv[1], std::stod(argv[2]));
  simulator.step_continuously(1000);
  simulator.save_json();
  return 0;
}
