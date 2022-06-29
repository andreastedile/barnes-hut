#ifndef BARNES_HUT_DESERIALIZATION_H
#define BARNES_HUT_DESERIALIZATION_H

#include "node.h"
#include "mpi_datatypes.h"

#include <memory>
#include <vector>

namespace bh {

std::unique_ptr<Node> deserialize_quadtree(const std::vector<mpi::Node> &nodes);

}

#endif  // BARNES_HUT_DESERIALIZATION_H
