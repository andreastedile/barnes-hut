#include <string>

#include "simple_simulator.h"

int main(int argc, char* argv[]) {
  bh::SimpleSimulator simulator(argv[1], std::stod(argv[2]));
  simulator.run_continuously(1);
  simulator.save();

  return 0;
}
