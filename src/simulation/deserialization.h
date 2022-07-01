#ifndef BARNES_HUT_DESERIALIZATION_H
#define BARNES_HUT_DESERIALIZATION_H

#include <memory>
#include <vector>

#include "body.h"
#include "mpi_datatypes.h"
#include "node.h"

namespace bh {

std::unique_ptr<Node> deserialize_quadtree(const std::vector<mpi::Node> &nodes);

Body deserialize(const mpi::Body &body);

std::vector<Body> deserialize(const std::vector<mpi::Body> &bodies);

}  // namespace bh

#endif  // BARNES_HUT_DESERIALIZATION_H
