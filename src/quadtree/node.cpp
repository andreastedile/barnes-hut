#include "node.h"

#include <string>

// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

#ifndef NDEBUG
#include <iostream>
#endif

#include <stdexcept>  // invalid_argument, runtime_error

namespace bh {

Node::Fork::AggregateBody compute_aggregate_body(const Node &nw, const Node &ne, const Node &se, const Node &sw) {
  Eigen::Vector2d center_of_mass = nw.center_of_mass() * nw.total_mass() +
                                   ne.center_of_mass() * ne.total_mass() +
                                   se.center_of_mass() * se.total_mass() +
                                   sw.center_of_mass() * sw.total_mass();
  double total_mass = nw.total_mass() + ne.total_mass() + se.total_mass() + sw.total_mass();

  return {center_of_mass / total_mass, total_mass};
}

Node::Leaf::Leaf(const Body &body) : m_body(body) {}

Node::Fork::Fork(std::unique_ptr<Node> nw, std::unique_ptr<Node> ne, std::unique_ptr<Node> se, std::unique_ptr<Node> sw, int n_nodes, AggregateBody aggregate_body)
    : m_nw(std::move(nw)), m_ne(std::move(ne)), m_se(std::move(se)), m_sw(std::move(sw)), m_n_nodes(n_nodes), m_aggregate_body(std::move(aggregate_body)) {}

Node::Node(const Eigen::Vector2d &bottom_left, const Eigen::Vector2d &top_right)
    : m_data(Leaf()), m_box(bottom_left, top_right) {
  if (m_box.sizes().x() != m_box.sizes().y()) {
    // clang-format off
    throw std::invalid_argument("Cannot create a non-square subquadrant. The following arguments were provided:\n"
        "min: (" + std::to_string(bottom_left.x()) + ", " + std::to_string(bottom_left.y()) + "),\n"
        "max: (" + std::to_string(top_right.x()) + ", " + std::to_string(top_right.y()) + "),\n"
        "length: " + std::to_string(m_box.sizes().x()) + ", height: " + std::to_string(m_box.sizes().y()));
    // clang-format on
  }
}

Node::Node(const Eigen::Vector2d &bottom_left, const Eigen::Vector2d &top_right, Fork fork)
    : m_box(bottom_left, top_right), m_data(std::move(fork)) {}

Node::Node(const Eigen::Vector2d &bottom_left, const Eigen::Vector2d &top_right, Leaf leaf)
    : m_box(bottom_left, top_right), m_data(std::move(leaf)) {}

void Node::insert(const Body &new_body) {
  auto visit_leaf = [&](Node::Leaf &leaf) {
    Subquadrant sq = get_subquadrant(new_body.m_position);
    if (sq == OUTSIDE) {
      throw std::invalid_argument(
          "Attempted to insert a new body outside of the node's bounding box");
    }

    if (leaf.m_body.has_value()) {
      Body &existing_body = *leaf.m_body;

      // If the two bodies coincide, sum their masses.
      if (new_body.m_position == existing_body.m_position) {
#ifndef NDEBUG
        std::cout << "bodies coincide\n";
#endif
        existing_body.m_mass += new_body.m_mass;
      }
      // Otherwise, create four empty subquadrants, relocate the existing body
      // and the new body in the corresponding subquadrants, and apply
      // recurison.
      else {
        auto nw = std::make_unique<Node>((m_box.corner(m_box.TopLeft) + m_box.corner(m_box.BottomLeft)) / 2, (m_box.corner(m_box.TopRight) + m_box.corner(m_box.TopLeft)) / 2);
        auto ne = std::make_unique<Node>(m_box.center(), m_box.corner(m_box.TopRight));
        auto se = std::make_unique<Node>((m_box.corner(m_box.BottomRight) + m_box.corner(m_box.BottomLeft)) / 2, (m_box.corner(m_box.TopRight) + m_box.corner(m_box.BottomRight)) / 2);
        auto sw = std::make_unique<Node>(m_box.corner(m_box.BottomLeft), m_box.center());

        // keep track of how many new quadtree nodes are created as part of the insertion process
        int n_nodes = 4;

        switch (get_subquadrant(existing_body.m_position)) {
          case NW:
            nw->insert(existing_body);
            break;
          case NE:
            ne->insert(existing_body);
            break;
          case SE:
            se->insert(existing_body);
            break;
          case SW:
            sw->insert(existing_body);
            break;
          case OUTSIDE:
            throw std::runtime_error(
                "An existing body is outside of the node's bounding box");
        }

        switch (sq) {
          case NW: {
            int n_nw_nodes = nw->n_nodes();
            nw->insert(new_body);
            int n_new_nw_nodes = nw->n_nodes() - n_nw_nodes;
            n_nodes += n_new_nw_nodes;
            break;
          }
          case NE: {
            int n_ne_nodes = ne->n_nodes();
            ne->insert(new_body);
            int n_new_ne_nodes = ne->n_nodes() - n_ne_nodes;
            n_nodes += n_new_ne_nodes;
            break;
          }
          case SE: {
            int n_se_nodes = se->n_nodes();
            se->insert(new_body);
            int n_se_new_nodes = se->n_nodes() - n_se_nodes;
            n_nodes += n_se_new_nodes;
            break;
          }
          case SW: {
            int n_sw_nodes = sw->n_nodes();
            sw->insert(new_body);
            int n_sw_new_nodes = sw->n_nodes() - n_sw_nodes;
            n_nodes += n_sw_new_nodes;
            break;
          }
          default:
            throw std::runtime_error("Reached default case in switch");
        }

        auto aggregate_body = compute_aggregate_body(*nw, *ne, *se, *sw);
        m_data = Fork(std::move(nw), std::move(ne), std::move(se), std::move(sw), n_nodes, std::move(aggregate_body));
      }
    } else {
      leaf.m_body = new_body;
    }
  };

  auto visit_fork = [&](Fork &fork) {
    switch (get_subquadrant(new_body.m_position)) {
      case NW: {
        int n_nw_nodes = fork.m_nw->n_nodes();
        fork.m_nw->insert(new_body);
        int n_new_nw_nodes = fork.m_nw->n_nodes() - n_nw_nodes;
        fork.m_n_nodes += n_new_nw_nodes;
        break;
      }
      case NE: {
        int n_ne_nodes = fork.m_ne->n_nodes();
        fork.m_ne->insert(new_body);
        int n_new_ne_nodes = fork.m_ne->n_nodes() - n_ne_nodes;
        fork.m_n_nodes += n_new_ne_nodes;
        break;
      }
      case SE: {
        int n_se_nodes = fork.m_se->n_nodes();
        fork.m_se->insert(new_body);
        int n_new_se_nodes = fork.m_se->n_nodes() - n_se_nodes;
        fork.m_n_nodes += n_new_se_nodes;
        break;
      }
      case SW: {
        int n_sw_nodes = fork.m_sw->n_nodes();
        fork.m_sw->insert(new_body);
        int n_new_sw_nodes = fork.m_sw->n_nodes() - n_sw_nodes;
        fork.m_n_nodes += n_new_sw_nodes;
        break;
      }
      case OUTSIDE:
        throw std::invalid_argument(
            "Attempted to insert a new body outside of the node's bounding "
            "box");
    }
    fork.m_aggregate_body = compute_aggregate_body(*fork.m_nw, *fork.m_ne, *fork.m_se, *fork.m_sw);
  };

  std::visit(overloaded{visit_fork, visit_leaf}, m_data);
}

Node::Subquadrant Node::get_subquadrant(const Eigen::Vector2d &point) {
  bool west = m_box.min().x() <= point.x() && point.x() < m_box.center().x();
  bool east = m_box.center().x() <= point.x() && point.x() <= m_box.max().x();
  bool south = m_box.min().y() <= point.y() && point.y() < m_box.center().y();
  bool north = m_box.center().y() <= point.y() && point.y() <= m_box.max().y();

  if (north && west)
    return NW;
  else if (north && east)
    return NE;
  else if (south && east)
    return SE;
  else if (south && west)
    return SW;
  else
    return OUTSIDE;
}

Eigen::Vector2d Node::center_of_mass() const {
  auto visit_leaf = [&](const Node::Leaf &leaf) -> Eigen::Vector2d {
    if (leaf.m_body.has_value()) {
      return leaf.m_body->m_position;
    }
    return {0, 0};
  };

  auto visit_fork = [&](const Node::Fork &fork) -> Eigen::Vector2d {
    return fork.m_aggregate_body.m_position;
  };

  return std::visit(overloaded{visit_fork, visit_leaf}, m_data);
}

double Node::total_mass() const {
  auto visit_leaf = [&](const Node::Leaf &leaf) -> double {
    if (leaf.m_body.has_value()) {
      return leaf.m_body->m_mass;
    }
    return 0;
  };

  auto visit_fork = [&](const Node::Fork &fork) -> double {
    return fork.m_aggregate_body.m_mass;
  };

  return std::visit(overloaded{visit_fork, visit_leaf}, m_data);
}

int Node::n_nodes() const {
  auto visit_fork = [](const Node::Fork &fork) {
    return 1 + fork.m_n_nodes;
  };

  auto visit_leaf = [](const Node::Leaf &) {
    return 1;
  };

  return std::visit(overloaded{visit_fork, visit_leaf}, m_data);
}

const Eigen::AlignedBox2d &Node::bbox() const { return m_box; }

const std::variant<Node::Fork, Node::Leaf> &Node::data() const {
  return m_data;
}

}  // namespace bh
