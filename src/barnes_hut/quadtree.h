#ifndef BARNES_HUT_QUADTREE_H
#define BARNES_HUT_QUADTREE_H

#include "node.h"
#include "body.h"

#include <memory>
#include <vector>

namespace bh {

std::unique_ptr<Node> construct_quadtree(const std::vector<Body> & bodies);

}

#endif  // BARNES_HUT_QUADTREE_H
