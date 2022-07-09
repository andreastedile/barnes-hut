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
    int nw_idx = idx + 1;
    int ne_idx = idx + 1 + visited.m_nw->n_nodes();
    int se_idx = idx + 1 + visited.m_nw->n_nodes() + visited.m_ne->n_nodes();
    int sw_idx = idx + 1 + visited.m_nw->n_nodes() + visited.m_ne->n_nodes() + visited.m_se->n_nodes();

    serialize_impl(*visited.m_nw, nodes, nw_idx);
    serialize_impl(*visited.m_ne, nodes, ne_idx);
    serialize_impl(*visited.m_se, nodes, se_idx);
    serialize_impl(*visited.m_sw, nodes, sw_idx);

    mpi::Node::Fork::AggregateBody aggregate_body = serialize_body(visited.m_aggregate_body);
    mpi::Node::Fork fork{nw_idx, ne_idx, se_idx, sw_idx, node.n_nodes(), aggregate_body};
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
