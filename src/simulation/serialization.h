#ifndef BARNES_HUT_SERIALIZATION_H
#define BARNES_HUT_SERIALIZATION_H

#include "body.h"
#include "mpi_datatypes.h"
#include "node.h"

namespace bh {

std::vector<mpi::Node> serialize(const Node& node);

mpi::Body serialize(const Body& body);

std::vector<mpi::Body> serialize(const std::vector<Body>& bodies);

}  // namespace bh

#endif  // BARNES_HUT_SERIALIZATION_H
