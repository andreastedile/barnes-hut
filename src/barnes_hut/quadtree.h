#ifndef BARNES_HUT_QUADTREE_H
#define BARNES_HUT_QUADTREE_H

#include <memory>
#include <vector>

#include "body.h"
#include "node.h"

namespace bh {

std::unique_ptr<Node> construct_quadtree(const std::vector<Body>& bodies);

std::unique_ptr<Node> merge_quadtrees(std::unique_ptr<Node> nw, std::unique_ptr<Node> ne, std::unique_ptr<Node> se, std::unique_ptr<Node> sw);

}  // namespace bh

#endif  // BARNES_HUT_QUADTREE_H
