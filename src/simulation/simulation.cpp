#include "simulation.h"

#include <algorithm>  // for_each, transform
#include <execution>  // par_unseq
#include <fstream>    // ifstream
#include <iostream>
#include <memory>     // make_shared
#include <stdexcept>  // runtime_error

#include "node.h"

namespace bh {

std::vector<SimulatedBody> load(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  std::cout << "Reading file...\n";

  unsigned n_bodies;
  file >> n_bodies;

  std::vector<SimulatedBody> bodies;
  bodies.reserve(n_bodies);
  for (unsigned i = 0; i < n_bodies; i++) {
    double position_x, position_y;
    double mass;
    double velocity_x, velocity_y;
    file >> position_x >> position_y;
    file >> mass;
    file >> velocity_x >> velocity_y;
    bodies.emplace_back(Vector2d{position_x, position_y}, mass,
                        Vector2d{velocity_x, velocity_y});
  }

  std::cout << "Read " << n_bodies << " bodies\n";
  return bodies;
}

std::tuple<std::vector<SimulatedBody>, AlignedBox2d> compute_new_bodies_exact(
    const std::vector<SimulatedBody> &bodies, const double dt) {
  // This calls SimulatedBody's default constructor bodies.size() times, but we
  // cannot do anything about it
#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  std::vector<SimulatedBody> new_bodies(bodies.size());
  std::transform(std::execution::par_unseq, bodies.begin(), bodies.end(),
                 new_bodies.begin(), [&](const SimulatedBody &body) {
                   return body.updated(bodies, dt);
                 });
#ifndef NDEBUG
  std::cout << "Computing new bounding box...\n";
#endif
  auto new_bbox = compute_square_bounding_box(new_bodies);

  return std::make_tuple(std::move(new_bodies), std::move(new_bbox));
}

std::tuple<std::vector<SimulatedBody>, AlignedBox2d,
           std::shared_ptr<const Node>>
compute_new_bodies_barnes_hut(const std::vector<SimulatedBody> &bodies,
                              const AlignedBox2d &bbox, double dt) {
#ifndef NDEBUG
  std::cout << "Computing quadtree...\n";
#endif
  auto quadtree = std::make_shared<Node>(bbox.min(), bbox.max());

  std::for_each(bodies.begin(), bodies.end(),
                [&](const SimulatedBody &simulated) {
                  Body body(simulated.m_position, simulated.m_mass);
                  quadtree->insert(body);
                });
  // This calls SimulatedBody's default constructor bodies.size() times, but we
  // cannot do anything about it
#ifndef NDEBUG
  std::cout << "Computing new bodies...\n";
#endif
  std::vector<SimulatedBody> new_bodies(bodies.size());
  std::transform(std::execution::par_unseq, bodies.begin(), bodies.end(),
                 new_bodies.begin(), [&](const SimulatedBody &body) {
                   auto new_body = body.updated(*quadtree, dt);
                   return new_body;
                 });

#ifndef NDEBUG
  std::cout << "Computing new  bounding box...\n";
#endif
  AlignedBox2d new_bbox = compute_square_bounding_box(new_bodies);

  return std::make_tuple(std::move(new_bodies), std::move(new_bbox),
                         std::move(quadtree));
}

}  // namespace bh