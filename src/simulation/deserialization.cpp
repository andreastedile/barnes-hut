#include "deserialization.h"

#include <eigen3/Eigen/Eigen>
#include <stdexcept>

using Eigen::Vector2d;

namespace bh {

std::unique_ptr<Node> deserialize_impl(const std::vector<mpi::Node> &nodes, int idx) {
  const mpi::Node &node = nodes[idx];

  switch (node.type) {
    case mpi::Node::ForkType: {
      std::array<std::unique_ptr<Node>, 4> children{
          deserialize_impl(nodes, node.data.fork.children[Node::NW]),
          deserialize_impl(nodes, node.data.fork.children[Node::NE]),
          deserialize_impl(nodes, node.data.fork.children[Node::SE]),
          deserialize_impl(nodes, node.data.fork.children[Node::SW])};

      Node::Fork::AggregateBody aggregate_body{{node.data.fork.aggregate_body.position_x, node.data.fork.aggregate_body.position_y}, node.data.fork.aggregate_body.mass};
      Node::Fork fork{std::move(children), node.data.fork.n_nodes, std::move(aggregate_body)};
      return std::make_unique<Node>(Vector2d{node.bottom_left_x, node.bottom_left_y},
                                    Vector2d{node.top_right_x, node.top_right_y},
                                    std::move(fork));
    }
    case mpi::Node::LeafType: {
      if (node.data.leaf.has_value) {
        Body body{{node.data.leaf.body.position_x, node.data.leaf.body.position_y}, node.data.leaf.body.mass};
        Node::Leaf leaf{body};
        return std::make_unique<Node>(Vector2d{node.bottom_left_x, node.bottom_left_y},
                                      Vector2d{node.top_right_x, node.top_right_y},
                                      leaf);
      } else {
        return std::make_unique<Node>(Vector2d{node.bottom_left_x, node.bottom_left_y},
                                      Vector2d{node.top_right_x, node.top_right_y},
                                      Node::Leaf{});
      }
    }
    default:
      throw std::runtime_error("reached default case");
  }
}

std::unique_ptr<Node> deserialize(const std::vector<mpi::Node> &nodes) {
  return deserialize_impl(nodes, 0);
}

}  // namespace bh