#include <mpi.h>

#include <algorithm>
#include <argparse/argparse.hpp>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "force.h"
#include "local_barnes_hut_simulator.h"
#include "local_exact_simulator.h"
#include "mpi_barnes_hut_simulator.h"

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
  app.add_argument("-output")
      .help("specify the output filename");
  app.add_argument("-type")
      .default_value(std::string("local-barnes-hut"))
      .action([](const std::string& type) {
        static const std::vector<std::string> choices{"local-barnes-hut", "mpi-barnes-hut", "local-exact"};
        if (std::find(choices.begin(), choices.end(), type) != choices.end()) {
          return type;
        } else {
          throw std::invalid_argument("invalid simulation type");
        }
      })
      .help("specify the simulation type");

  try {
    app.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    // std::cerr << err.what() << '\n';
    std::cerr << app;
    std::exit(1);
  }

  std::unique_ptr<bh::ISimulation> simulator;
  const auto type = app.get<std::string>("type");
  if (type == "local-barnes-hut") {
    simulator = std::make_unique<bh::LocalBarnesHutSimulator>(
        app.get<std::string>("input"),
        app.get<double>("dt"),
        app.get<double>("-G"),
        app.get<double>("-omega"));
  } else if (type == "local-exact") {
    simulator = std::make_unique<bh::LocalExactSimulator>(
        app.get<std::string>("input"),
        app.get<double>("dt"),
        app.get<double>("-G"));
  } else if (type == "mpi-barnes-hut") {
    MPI_Init(nullptr, nullptr);

    int proc_id, n_procs;
    MPI_Comm_rank(MPI_COMM_WORLD, &proc_id);
    MPI_Comm_size(MPI_COMM_WORLD, &n_procs);

    auto is_power_of_four = [&](int n) {
      if (n == 0)
        return 0;
      while (n != 1) {
        if (n % 4 != 0)
          return 0;
        n = n / 4;
      }
      return 1;
    };
//    if (is_power_of_four(n_procs) == 1) {
//      throw std::invalid_argument("number of processors must be a power of 4!");
//    }

    simulator = std::make_unique<bh::MpiBarnesHutSimulator>(
        bh::MpiBarnesHutSimulator::from_file(proc_id,
                                             n_procs,
                                             app.get<std::string>("input"),
                                             app.get<double>("dt"),
                                             app.get<double>("-G"),
                                             app.get<double>("-omega")));
  }

  simulator->step_continuously(app.get<int>("steps"));

  std::cout << "Simulation completed\n";

  if (auto output = app.present("-output")) {
    std::cout << "Saving JSON to " << *output << "...\n";
    simulator->save(*output);
  }

  if (type == "mpi-barnes-hut") {
    MPI_Finalize();
  }

  return 0;
}
