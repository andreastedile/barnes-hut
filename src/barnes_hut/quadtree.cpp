#include "quadtree.h"

#include <algorithm>  // transform
#include <execution>  // par_unseq

namespace bh {

std::unique_ptr<Node> construct_quadtree(const std::vector<Body>& bodies) {
  auto bbox = compute_square_bounding_box(bodies);
  auto quadtree = std::make_unique<Node>(bbox.min(), bbox.max());
  std::for_each(bodies.begin(), bodies.end(), [&quadtree](const Body& body) { quadtree->insert(body); });
  return quadtree;
}

}  // namespace bh
