#include <argparse/argparse.hpp>
#include <fstream>
#include <string>
#include <utility>

#include "loader.h"
#include "src/exact_simulator.h"

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

  auto initial_bodies = bh::load_bodies(app.get("input"));

  bh::ExactSimulator simulator(app.get<double>("dt"),
                           app.get<double>("-G"),
                           std::move(initial_bodies));

  auto output = app.present("output");

  for (int i = 0; i < app.get<int>("steps"); i++) {
    const auto& step = simulator.step();

    if (output) {
      nlohmann::json j = step;
      std::fstream o(*output + std::to_string(i));
      o << j;
    }
  }

  return 0;
}
