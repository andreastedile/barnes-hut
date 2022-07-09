#include <argparse/argparse.hpp>
#include <iostream>
#include <string>
#include <utility>

#include "barnes_hut_simulation_step.h"
#include "bounding_box.h"
#include "loader.h"
#include "persistence.h"
#include "src/barnes_hut_simulator.h"

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
      .help("specify the simulation dt");
  app.add_argument("-G")
      .scan<'g', double>()
      .default_value(0.000000000066743)
      .help("specify the simulation gravitational constant");
  app.add_argument("-theta")
      .scan<'g', double>()
      .default_value(0.5)
      .help("specify the barnes–hut theta");
  app.add_argument("--no-output")
      .default_value(false)
      .implicit_value(true)
      .help("disables   saving the simulation steps file");

  try {
    app.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << app;
    std::exit(1);
  }

  const auto dt = app.get<double>("dt");
  const auto G = app.get<double>("-G");
  const auto theta = app.get<double>("-theta");
  const auto no_output = app.get<bool>("--no-output");

  auto initial_bodies = bh::load_bodies(app.get("input"));

  auto last_step = bh::BarnesHutSimulationStep(std::move(initial_bodies), bh::compute_square_bounding_box(initial_bodies));
  if (!no_output) {
    bh::write_to_file(last_step, "step0.json");
  }

  for (int i = 0; i < app.get<int>("steps"); i++) {
    std::cout << "Step " << i + 1 << '\n';

    last_step = bh::step(last_step, dt, G, theta);

    if (!no_output) {
      bh::write_to_file(last_step, "step" + std::to_string(i + 1) + ".json");
    }
  }

  std::cout << "Done. Exiting...\n";

  return 0;
}
