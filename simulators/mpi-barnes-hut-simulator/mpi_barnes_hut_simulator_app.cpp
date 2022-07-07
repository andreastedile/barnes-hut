#include <mpi.h>

#include <argparse/argparse.hpp>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <iostream>

#include "bounding_box.h"
#include "loader.h"
#include "power_of_four.h"
#include "src/mpi_barnes_hut_simulator.h"

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
  app.add_argument("-theta")
      .scan<'g', double>()
      .default_value(0.5)
      .help("specify the barnes–hut theta");
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
  const auto theta = app.get<double>("-theta");
  const auto output = app.present("output");

  MPI_Init(nullptr, nullptr);

  int proc_id, n_procs;
  MPI_Comm_rank(MPI_COMM_WORLD, &proc_id);
  MPI_Comm_size(MPI_COMM_WORLD, &n_procs);

  if (!is_power_of_four(n_procs)) {
    throw std::runtime_error("The number of processors of the MPI communicator must be a power of 4 (found: " + std::to_string(n_procs) + ')');
  }

  auto initial_bodies = bh::load_bodies(app.get("input"));

  auto last_step = bh::BarnesHutSimulationStep(std::move(initial_bodies), bh::compute_square_bounding_box(initial_bodies));

  for (int i = 0; i < app.get<int>("steps"); i++) {
    std::cout << "Step " << i + 1 << '\n';

    last_step = bh::step(last_step, dt, G, theta, proc_id, n_procs);

    if (output && proc_id == 0) {
      nlohmann::json j = last_step;
      std::ofstream o(output.value() + std::to_string(i) + ".json");
      o << j;
      o.close();
    }
  }

  MPI_Finalize();

  std::cout << "Done. Exiting...\n";

  return 0;
}
