#ifndef BARNES_HUT_QUADTREE_DESERIALIZATION_H
#define BARNES_HUT_QUADTREE_DESERIALIZATION_H

#include <memory>
#include <vector>

#include "mpi_datatypes.h"
#include "node.h"

namespace bh {

using QuadtreeGrid = std::vector<std::vector<std::unique_ptr<Node>>>;

QuadtreeGrid deserialize_quadtrees(int n_procs, const std::vector<mpi::Node> &quadtrees, const std::vector<int> &n_nodes);

std::unique_ptr<Node> deserialize_quadtree(const std::vector<mpi::Node> &nodes);

}  // namespace bh

#endif  // BARNES_HUT_QUADTREE_DESERIALIZATION_H
