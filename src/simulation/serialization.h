#ifndef BARNES_HUT_SERIALIZATION_H
#define BARNES_HUT_SERIALIZATION_H

#include "node.h"
#include "mpi_datatypes.h"

namespace bh {

std::vector<mpi::Node> serialize(const Node & node);

}

#endif  // BARNES_HUT_SERIALIZATION_H
