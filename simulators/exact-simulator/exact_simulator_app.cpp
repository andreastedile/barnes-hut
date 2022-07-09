#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>
#include <string>
#include <utility>

#include "bounding_box.h"
#include "loader.h"
#include "persistence.h"
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

  const auto input = app.get("input");
  const auto steps = app.get<int>("steps");
  const auto dt = app.get<double>("dt");
  const auto G = app.get<double>("-G");
  const auto no_output = app.get<bool>("--no-output");

  auto initial_bodies = bh::load_bodies(input);

  auto last_step = bh::SimulationStep(std::move(initial_bodies), bh::compute_square_bounding_box(initial_bodies));
  if (!no_output) {
    bh::write_to_file(last_step, "step0.json");
  }

  spdlog::set_pattern("[%H:%M:%S:%f] %v");

  for (int i = 1; i <= steps; i++) {
    spdlog::info("Step {}", i);

    last_step = bh::step(last_step, dt, G);

    if (!no_output) {
      bh::write_to_file(last_step, "step" + std::to_string(i) + ".json");
    }
  }

  spdlog::info("Done. Exiting...");

  return 0;
}
