#include "quadtree_serialization.h"
#include "body_serialization.h"

namespace bh {

// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

/**
 * @param node of the quadtree that the recursion is currently visiting
 * @param nodes vector in which to write the visited node
 * @param idx index of the nodes vector in which to write the visited node
 */
void serialize_impl(const Node& node, std::vector<mpi::Node>& nodes, int idx) {
  auto visit_fork = [&](const Node::Fork& visited) {
    int children[4]{idx + 1,
                    idx + 1 + visited.m_children[Node::NW]->n_nodes(),
                    idx + 1 + visited.m_children[Node::NW]->n_nodes() + visited.m_children[Node::NE]->n_nodes(),
                    idx + 1 + visited.m_children[Node::NW]->n_nodes() + visited.m_children[Node::NE]->n_nodes() + visited.m_children[Node::SE]->n_nodes()};

    serialize_impl(*visited.m_children[Node::NW], nodes, children[Node::NW]);
    serialize_impl(*visited.m_children[Node::NE], nodes, children[Node::NE]);
    serialize_impl(*visited.m_children[Node::SE], nodes, children[Node::SE]);
    serialize_impl(*visited.m_children[Node::SW], nodes, children[Node::SW]);

    mpi::Node::Fork::AggregateBody aggregate_body = serialize_body(visited.m_aggregate_body);
    mpi::Node::Fork fork{children, node.n_nodes(), aggregate_body};
    nodes[idx] = mpi::Node{fork, node.bbox().min().x(), node.bbox().min().y(), node.bbox().max().x(), node.bbox().max().y()};
  };
  auto visit_leaf = [&](const Node::Leaf& visited) {
    if (visited.m_body.has_value()) {
      mpi::Body body = serialize_body(*visited.m_body);
      mpi::Node::Leaf leaf{body};
      nodes[idx] = mpi::Node(leaf, node.bbox().min().x(), node.bbox().min().y(), node.bbox().max().x(), node.bbox().max().y());
    } else {
      mpi::Node::Leaf leaf{};
      nodes[idx] = mpi::Node(leaf, node.bbox().min().x(), node.bbox().min().y(), node.bbox().max().x(), node.bbox().max().y());
    }
  };

  std::visit(overloaded{visit_fork, visit_leaf}, node.data());
}

std::vector<mpi::Node> serialize_quadtree(const Node& node) {
  std::vector<mpi::Node> nodes(node.n_nodes());
  serialize_impl(node, nodes, 0);
  return nodes;
}

}
