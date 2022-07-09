#include "quadtree.h"

#include <algorithm>  // transform, count_if
#include <array>
#include <utility>  // move
#include <variant>  // holds_alternative

#include "bounding_box.h"

namespace bh {

std::unique_ptr<Node> construct_quadtree(const std::vector<Body>& bodies, const Eigen::AlignedBox2d& bbox) {
  auto quadtree = std::make_unique<Node>(bbox.min(), bbox.max());
  std::for_each(bodies.begin(), bodies.end(), [&quadtree](const Body& body) { quadtree->insert(body); });
  return quadtree;
}

std::unique_ptr<Node> construct_quadtree(const std::vector<Body>& bodies) {
  auto bbox = compute_square_bounding_box(bodies);
  return construct_quadtree(bodies, bbox);
}

std::unique_ptr<Node> merge_quadtrees(std::unique_ptr<Node> nw, std::unique_ptr<Node> ne, std::unique_ptr<Node> se, std::unique_ptr<Node> sw) {
  int n_nodes = nw->n_nodes() + ne->n_nodes() + se->n_nodes() + sw->n_nodes();

  if (n_nodes == 4) {
    // if all children are leaves, we can attempt some optimizations
    auto nw_leaf = std::get<Node::Leaf>(nw->data());
    auto ne_leaf = std::get<Node::Leaf>(ne->data());
    auto se_leaf = std::get<Node::Leaf>(se->data());
    auto sw_leaf = std::get<Node::Leaf>(sw->data());

    // if all laves are empty, we can return an empty leaf node instead of a fork
    if (!nw_leaf.m_body && !ne_leaf.m_body && !se_leaf.m_body && !sw_leaf.m_body) {
      Node::Leaf leaf{};
      return std::make_unique<Node>(sw->bbox().min(), ne->bbox().max(), leaf);
    }

    // if there is only one body, we can return a leaf node containing that body instead of a fork
    if (nw_leaf.m_body && !ne_leaf.m_body && !se_leaf.m_body && !sw_leaf.m_body) {
      Node::Leaf leaf(nw_leaf.m_body.value());
      return std::make_unique<Node>(sw->bbox().min(), ne->bbox().max(), leaf);
    } else if (!nw_leaf.m_body && ne_leaf.m_body && !se_leaf.m_body && !sw_leaf.m_body) {
      Node::Leaf leaf(ne_leaf.m_body.value());
      return std::make_unique<Node>(sw->bbox().min(), ne->bbox().max(), leaf);
    } else if (!nw_leaf.m_body && !ne_leaf.m_body && se_leaf.m_body && !sw_leaf.m_body) {
      Node::Leaf leaf(se_leaf.m_body.value());
      return std::make_unique<Node>(sw->bbox().min(), ne->bbox().max(), leaf);
    } else if (!nw_leaf.m_body && !ne_leaf.m_body && !se_leaf.m_body && sw_leaf.m_body) {
      Node::Leaf leaf(sw_leaf.m_body.value());
      return std::make_unique<Node>(sw->bbox().min(), ne->bbox().max(), leaf);
    }
  }

  // regardless of whether the nodes are fork or leaves, they contain more than one body

  auto aggregate_body = compute_aggregate_body(*nw, *ne, *se, *sw);
  auto bottom_left = sw->bbox().min();
  auto top_right = ne->bbox().max();
  Node::Fork fork(std::move(nw), std::move(ne), std::move(se), std::move(sw), n_nodes, aggregate_body);
  return std::make_unique<Node>(std::move(bottom_left), std::move(top_right), std::move(fork));
}

std::unique_ptr<Node> reconstruct_quadtree(QuadtreeGrid& matrix) {
  const int N_ROWS = static_cast<int>(matrix.size());
  const int N_COLS = N_ROWS;

  // N_COLS == 1 would be equivalent
  if (N_ROWS == 1) {
    return std::move(matrix[0][0]);
  }

  const int N_NEW_ROWS = N_ROWS / 2;
  const int N_NEW_COLS = N_NEW_ROWS;

  QuadtreeGrid new_matrix(N_NEW_ROWS);
  for (int i = 0; i < N_NEW_ROWS; i++) {
    new_matrix[i].resize(N_NEW_COLS);
  }

  for (int i = 0; i < N_ROWS; i += 2) {
    for (int j = 0; j < N_COLS; j += 2) {
      const int N = i;
      const int S = i + 1;
      const int W = j;
      const int E = j + 1;
      auto new_node = merge_quadtrees(std::move(matrix[N][W]), std::move(matrix[N][E]), std::move(matrix[S][E]), std::move(matrix[S][W]));
      new_matrix[i / 2][j / 2] = std::move(new_node);
    }
  }

  return reconstruct_quadtree(new_matrix);
}

}  // namespace bh
