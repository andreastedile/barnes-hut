#include <mpi.h>

#include <argparse/argparse.hpp>
#include <fstream>
#include <string>
#include <utility>
#include <stdexcept>

#include "loader.h"
#include "src/mpi_barnes_hut_simulator.h"
#include "power_of_four.h"

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

  MPI_Init(nullptr, nullptr);

  int proc_id, n_procs;
  MPI_Comm_rank(MPI_COMM_WORLD, &proc_id);
  MPI_Comm_size(MPI_COMM_WORLD, &n_procs);

  if (!is_power_of_four(n_procs)) {
    throw std::runtime_error("The number of processors of the MPI communicator must be a power of 4 (found: " + std::to_string(n_procs) + ')');
  }

  auto initial_bodies = bh::load_bodies(app.get("input"));

  bh::MpiBarnesHutSimulator simulator(app.get<double>("dt"),
                                      app.get<double>("-G"),
                                      app.get<double>("-theta"),
                                      std::move(initial_bodies),
                                      proc_id, n_procs);

  auto output = app.present("output");

  for (int i = 0; i < app.get<int>("steps"); i++) {
    const auto& step = simulator.step();

    if (output) {
      nlohmann::json j = step;
      std::fstream o(*output + std::to_string(i));
      o << j;
    }
  }

  MPI_Finalize();

  return 0;
}
