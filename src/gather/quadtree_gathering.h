#ifndef BARNES_HUT_QUADTREE_GATHERING_H
#define BARNES_HUT_QUADTREE_GATHERING_H

#include <vector>

#include "node.h"

namespace bh {

std::unique_ptr<Node> gather_quadtree(int proc_id, int n_procs, const Node& my_quadtree);

}

#endif  // BARNES_HUT_QUADTREE_GATHERING_H
