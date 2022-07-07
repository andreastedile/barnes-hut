#include <argparse/argparse.hpp>
#include <fstream>
#include <string>
#include <utility>

#include "loader.h"
#include "src/exact_simulator.h"
#include "bounding_box.h"

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
      .default_value(0.000000000066743)
      .help("specify the simulation dt");
  app.add_argument("-output")
      .help("specify the output filename");

  try {
    app.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << app;
    std::exit(1);
  }

  const auto dt = app.get<double>("dt");
  const auto G = app.get<double>("-G");
  const auto output = app.present("output");

  auto initial_bodies = bh::load_bodies(app.get("input"));

  auto last_step = bh::SimulationStep(std::move(initial_bodies), bh::compute_square_bounding_box(initial_bodies));

  for (int i = 0; i < app.get<int>("steps"); i++) {
    last_step = bh::step(last_step, dt, G);

    if (output) {
      nlohmann::json j = last_step;
      std::ofstream o(output.value() + std::to_string(i) + ".json");
      o << j;
      o.close();
    }
  }

  return 0;
}
