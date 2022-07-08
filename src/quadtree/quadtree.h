#ifndef BARNES_HUT_QUADTREE_H
#define BARNES_HUT_QUADTREE_H

#include <memory>
#include <vector>
#include <Eigen/Geometry>

#include "body.h"
#include "node.h"

namespace bh {

using QuadtreeGrid = std::vector<std::vector<std::unique_ptr<Node>>>;

std::unique_ptr<Node> construct_quadtree(const std::vector<Body>& bodies, const Eigen::AlignedBox2d& bbox);

std::unique_ptr<Node> merge_quadtrees(std::unique_ptr<Node> nw, std::unique_ptr<Node> ne, std::unique_ptr<Node> se, std::unique_ptr<Node> sw);

std::unique_ptr<Node> reconstruct_quadtree(QuadtreeGrid& matrix);

}  // namespace bh

#endif  // BARNES_HUT_QUADTREE_H
