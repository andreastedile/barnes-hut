#ifndef BARNES_HUT_QUADTREE_SERIALIZATION_H
#define BARNES_HUT_QUADTREE_SERIALIZATION_H

#include <vector>

#include "mpi_datatypes.h"
#include "node.h"

namespace bh {

std::vector<mpi::Node> serialize_quadtree(const Node& node);

}

#endif  // BARNES_HUT_QUADTREE_SERIALIZATION_H
