#include <argparse/argparse.hpp>
#include <iostream>
#include <string>

#include "force.h"
#include "local_barnes_hut_simulator.h"

int main(int argc, char* argv[]) {
  argparse::ArgumentParser app("Barnes–Hut simulation");
  app.add_argument("input")
      .required()
      .help("specify the input file");
  app.add_argument("steps")
      .scan<'d', int>()
      .required()
      .help("specify the number of simulation steps");
  app.add_argument("dt")
      .scan<'g', double>()
      .required()
      .help("specify the simulation gravitational constant");
  app.add_argument("-G")
      .scan<'g', double>()
      .default_value(bh::NEWTONIAN_G)
      .help("specify the simulation dt");
  app.add_argument("-omega")
      .scan<'g', double>()
      .default_value(bh::DEFAULT_OMEGA)
      .help("specify the simulation omega");
  try {
    app.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    // std::cerr << err.what() << '\n';
    std::cerr << app;
    std::exit(1);
  }

  bh::LocalBarnesHutSimulator simulator(
      app.get<std::string>("input"),
      app.get<double>("dt"),
      app.get<double>("-G"),
      app.get<double>("-omega"));
  simulator.step_continuously(app.get<int>("steps"));

  std::cout << "Simulation completed. Saving JSON...\n";
  simulator.save();
  return 0;
}
