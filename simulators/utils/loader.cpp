#include "loader.h"

#include <algorithm>           // copy_n
#include <eigen3/Eigen/Eigen>  // Vector2d
#include <fstream>             // ifstream
#include <iostream>            // cout
#include <iterator>            // istream_iterator, back_inserter
#include <stdexcept>           // runtime_error
#include <string>              // to_string

namespace bh {

std::vector<Body> load_bodies(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  int n_bodies;
  file >> n_bodies;
  if (n_bodies < 0) {
    throw std::runtime_error("Number of bodies must be non-negative (read " + std::to_string(n_bodies) + ')');
  }

  std::cout << "Reading " << n_bodies << " from " << filename << "...\n";

  std::vector<Body> bodies;
  bodies.reserve(n_bodies);
  std::copy_n(std::istream_iterator<Body>{file}, n_bodies, std::back_inserter(bodies));

  std::cout << "Read " << n_bodies << " bodies\n";
  return bodies;
}

}  // namespace bh
