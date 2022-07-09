#include "quadtree_deserialization.h"

#include <Eigen/Eigen>
#include <array>
#include <functional>
#include <memory>
#include <stdexcept>  // runtime_error
#include <utility>

namespace bh {

std::unique_ptr<Node> deserialize_quadtree_impl(const std::vector<mpi::Node> &nodes, int idx) {
  const mpi::Node &node = nodes[idx];

  switch (node.type) {
    case mpi::Node::ForkType: {
      auto nw = deserialize_quadtree_impl(nodes, node.data.fork.nw_idx);
      auto ne = deserialize_quadtree_impl(nodes, node.data.fork.ne_idx);
      auto se = deserialize_quadtree_impl(nodes, node.data.fork.se_idx);
      auto sw = deserialize_quadtree_impl(nodes, node.data.fork.sw_idx);

      Node::Fork::AggregateBody aggregate_body{{node.data.fork.aggregate_body.position_x, node.data.fork.aggregate_body.position_y}, node.data.fork.aggregate_body.mass};
      Node::Fork fork{std::move(nw), std::move(ne), std::move(se), std::move(sw), node.data.fork.n_nodes, std::move(aggregate_body)};
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

std::unique_ptr<Node> deserialize_quadtree_node(const mpi::Node &quadtree_node) {
  return deserialize_quadtree_impl({quadtree_node}, 0);
}

std::unique_ptr<Node> deserialize_quadtree(const std::vector<mpi::Node> &quadtree_nodes) {
  return deserialize_quadtree_impl(quadtree_nodes, 0);
}

QuadtreeGrid deserialize_quadtrees(int n_procs, const std::vector<mpi::Node> &quadtrees, const std::vector<int> &n_nodes) {
  const int N_ROWS = static_cast<int>(std::sqrt(n_procs));
  const int N_COLS = N_ROWS;

  QuadtreeGrid matrix(N_ROWS);
  for (int i = 0; i < N_ROWS; i++) {
    matrix[i].resize(N_COLS);
  }

  std::vector<int> displacements(n_procs);
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
