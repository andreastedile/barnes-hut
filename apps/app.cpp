#include <string>

#include "simulation.h"

int main(int argc, char* argv[]) {
  bh::SimpleSimulator simulator(argv[1], std::stod(argv[2]), bh::APPROXIMATED);
  simulator.run_continuously(1);
  simulator.save();

  return 0;
}
