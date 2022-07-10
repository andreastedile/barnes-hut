#include <argparse/argparse.hpp>
#include <string>
#include <utility>
#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>
#include <fstream>
#include <iostream>

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
  app.add_argument("sampling_rate")
      .scan<'d', int>()
      .help("specify the sampling rate");
  app.add_argument("--no-output")
      .default_value(false)
      .implicit_value(true)
      .help("disables saving the simulation steps file");
  app.add_argument("-timings")
      .help("specify the output filename");

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
  const auto theta = app.get<double>("-theta");
  const auto sampling_rate = app.get<int>("sampling_rate");
  const auto no_output = app.get<bool>("--no-output");
  const auto timings = app.present("timings");

  auto initial_bodies = bh::load_bodies(input);

  auto last_step = bh::BarnesHutSimulationStep(std::move(initial_bodies), bh::compute_square_bounding_box(initial_bodies));
  if (!no_output) {
    bh::write_to_file(last_step, "step0.json");
  }

  spdlog::cfg::load_env_levels();
  spdlog::set_pattern("proc %P/thread %t elapsed: %i μs  [%l] %v");

  for (int i = 1; i <= steps; i++) {
    spdlog::info("Step {}", i);

    last_step = bh::step(last_step, dt, G, theta);

    if (!no_output && i % sampling_rate == 0) {
      bh::write_to_file(last_step, "step" + std::to_string(i) + ".json");
    }
  }

  std::cout << bh::timings();

  if (timings) {
    std::ofstream o(timings.value());
    o << bh::timings();
    o.close();
  }

  return 0;
}
