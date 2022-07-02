#include "quadtree_deserialization.h"

#include <array>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <memory>
#include <stdexcept>  // runtime_error
#include <utility>

namespace bh {

std::unique_ptr<Node> deserialize_quadtree_impl(const std::vector<mpi::Node> &nodes, int idx) {
  const mpi::Node &node = nodes[idx];

  switch (node.type) {
    case mpi::Node::ForkType: {
      std::array<std::unique_ptr<Node>, 4> children{
          deserialize_quadtree_impl(nodes, node.data.fork.children[Node::NW]),
          deserialize_quadtree_impl(nodes, node.data.fork.children[Node::NE]),
          deserialize_quadtree_impl(nodes, node.data.fork.children[Node::SE]),
          deserialize_quadtree_impl(nodes, node.data.fork.children[Node::SW])};

      Node::Fork::AggregateBody aggregate_body{{node.data.fork.aggregate_body.position_x, node.data.fork.aggregate_body.position_y}, node.data.fork.aggregate_body.mass};
      Node::Fork fork{std::move(children), node.data.fork.n_nodes, std::move(aggregate_body)};
      return std::make_unique<Node>(Eigen::Vector2d{node.bottom_left_x, node.bottom_left_y},
                                    Eigen::Vector2d{node.top_right_x, node.top_right_y},
                                    std::move(fork));
    }
    case mpi::Node::LeafType: {
      if (node.data.leaf.has_value) {
        Body body{{node.data.leaf.body.position_x, node.data.leaf.body.position_y}, node.data.leaf.body.mass};
        Node::Leaf leaf{body};
        return std::make_unique<Node>(Eigen::Vector2d{node.bottom_left_x, node.bottom_left_y},
                                      Eigen::Vector2d{node.top_right_x, node.top_right_y},
                                      leaf);
      } else {
        return std::make_unique<Node>(Eigen::Vector2d{node.bottom_left_x, node.bottom_left_y},
                                      Eigen::Vector2d{node.top_right_x, node.top_right_y},
                                      Node::Leaf{});
      }
    }
    default:
      throw std::runtime_error("reached default case");
  }
}

std::unique_ptr<Node> deserialize_quadtree(const std::vector<mpi::Node> &nodes) {
  return deserialize_quadtree_impl(nodes, 0);
}

QuadtreeGrid deserialize_quadtrees(const std::vector<mpi::Node> &quadtrees, const std::vector<int> &n_nodes) {
  const int N_SUBQUADRANTS = static_cast<int>(quadtrees.size());
  const int N_ROWS = static_cast<int>(std::sqrt(N_SUBQUADRANTS));
  const int N_COLS = N_ROWS;

  QuadtreeGrid matrix(N_ROWS);
  for (int i = 0; i < N_ROWS; i++) {
    matrix[i].resize(N_COLS);
  }

  std::vector<int> displacements(N_SUBQUADRANTS);
  std::partial_sum(n_nodes.begin(), n_nodes.end(),
                   displacements.begin() + 1,  // first displacement will be 0
                   std::plus<>());

  for (auto i = 0; i < N_ROWS; i++) {
    for (auto j = 0; j < N_COLS; j++) {
      const int idx_from = displacements[i * N_COLS + j];
      const int n_nodes_in_branch = n_nodes[i * N_COLS + j];
      const std::vector<mpi::Node> branch{quadtrees.begin() + idx_from, quadtrees.begin() + idx_from + n_nodes_in_branch};
      matrix[i][j] = deserialize_quadtree(branch);
    }
  }

  return matrix;
}

}  // namespace bh
