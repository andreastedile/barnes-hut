#include "node.h"

#include "templates.h"

#ifndef NDEBUG
#include <iostream>
#endif

#include <stdexcept>  // invalid_argument, runtime_error

namespace bh {

Node::Fork::AggregateBody compute_aggregate_body(const Node &nw,const Node &ne,const Node &se,const Node &sw) {
  Vector2d center_of_mass = nw.center_of_mass() * nw.total_mass() +
                            ne.center_of_mass() * ne.total_mass() +
                            se.center_of_mass() * se.total_mass() +
                            sw.center_of_mass() * sw.total_mass();
  double total_mass = nw.total_mass() + ne.total_mass() + se.total_mass() + sw.total_mass();

  return {center_of_mass / total_mass, total_mass};
}

Node::Leaf::Leaf(const Body &body) : m_body(body) {}

Node::Fork::Fork(std::array<std::unique_ptr<Node>, 4> children, int n_nodes, AggregateBody aggregate_body)
    : m_children(std::move(children)),
      m_n_nodes(n_nodes),
      m_aggregate_body(std::move(aggregate_body)) {}

void Node::Fork::update_aggregate_body() {
  m_aggregate_body = compute_aggregate_body(*m_children[Node::Subquadrant::NW],
                                            *m_children[Node::Subquadrant::NE],
                                            *m_children[Node::Subquadrant::SE],
                                            *m_children[Node::Subquadrant::SW]);
}

Node::Node(const Vector2d &bottom_left, const Vector2d &top_right)
    : m_data(Leaf()), m_box(bottom_left, top_right) {
  Vector2d top_left(bottom_left.x(), top_right.y());
  Vector2d bottom_right(top_right.x(), bottom_left.y());
  if ((top_left - bottom_left).norm() != (bottom_right - bottom_left).norm()) {
    throw std::invalid_argument("Cannot create a non-square subquadrant");
  }
}

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
        std::array<std::unique_ptr<Node>, 4> children{
            std::make_unique<Node>((m_box.corner(m_box.TopLeft) + m_box.corner(m_box.BottomLeft)) / 2, (m_box.corner(m_box.TopRight) + m_box.corner(m_box.TopLeft)) / 2),
            std::make_unique<Node>(m_box.center(), m_box.corner(m_box.TopRight)),
            std::make_unique<Node>((m_box.corner(m_box.BottomRight) + m_box.corner(m_box.BottomLeft)) / 2, (m_box.corner(m_box.TopRight) + m_box.corner(m_box.BottomRight)) / 2),
            std::make_unique<Node>(m_box.corner(m_box.BottomLeft), m_box.center())};

        // keep track of how many new quadtree nodes are created as part of the insertion process
        int n_nodes = 4;

        switch (get_subquadrant(existing_body.m_position)) {
          case NW:
            children[NW]->insert(existing_body);
            break;
          case NE:
            children[NE]->insert(existing_body);
            break;
          case SE:
            children[SE]->insert(existing_body);
            break;
          case SW:
            children[SW]->insert(existing_body);
            break;
          case OUTSIDE:
            throw std::runtime_error(
                "An existing body is outside of the node's bounding box");
        }

        switch (sq) {
          case NW: {
            int n_nw_nodes = children[NW]->n_nodes();
            children[NW]->insert(new_body);
            int n_new_nw_nodes = children[NW]->n_nodes() - n_nw_nodes;
            n_nodes += n_new_nw_nodes;
            break;
          }
          case NE: {
            int n_ne_nodes = children[NE]->n_nodes();
            children[NE]->insert(new_body);
            int n_new_ne_nodes = children[NE]->n_nodes() - n_ne_nodes;
            n_nodes += n_new_ne_nodes;
            break;
          }
          case SE: {
            int n_se_nodes = children[SE]->n_nodes();
            children[SE]->insert(new_body);
            int n_se_new_nodes = children[SE]->n_nodes() - n_se_nodes;
            n_nodes += n_se_new_nodes;
            break;
          }
          case SW: {
            int n_sw_nodes = children[SW]->n_nodes();
            children[SW]->insert(new_body);
            int n_sw_new_nodes = children[SW]->n_nodes() - n_sw_nodes;
            n_nodes += n_sw_new_nodes;
            break;
          }
          default:
            throw std::runtime_error("Reached default case in switch");
        }

        auto aggregate_body = compute_aggregate_body(*children[Node::NW], *children[Node::NW], *children[Node::NW],*children[Node::NW]);
        m_data = Fork(std::move(children), n_nodes, std::move(aggregate_body));
      }
    } else {
      leaf.m_body = new_body;
    }
  };

  auto visit_fork = [&](Fork &fork) {
    switch (get_subquadrant(new_body.m_position)) {
      case NW: {
        int n_nw_nodes = fork.m_children[NW]->n_nodes();
        Vector2d nw_center_of_mass = fork.m_children[NW]->center_of_mass();
        fork.m_children[NW]->insert(new_body);
        int n_new_nw_nodes = fork.m_children[NW]->n_nodes() - n_nw_nodes;
        fork.m_n_nodes += n_new_nw_nodes;
        break;
      }
      case NE: {
        int n_ne_nodes = fork.m_children[NE]->n_nodes();
        fork.m_children[NE]->insert(new_body);
        int n_new_ne_nodes = fork.m_children[NE]->n_nodes() - n_ne_nodes;
        fork.m_n_nodes += n_new_ne_nodes;
        break;
      }
      case SE: {
        int n_se_nodes = fork.m_children[SE]->n_nodes();
        fork.m_children[SE]->insert(new_body);
        int n_new_se_nodes = fork.m_children[SE]->n_nodes() - n_se_nodes;
        fork.m_n_nodes += n_new_se_nodes;
        break;
      }
      case SW: {
        int n_sw_nodes = fork.m_children[SW]->n_nodes();
        fork.m_children[SW]->insert(new_body);
        int n_new_sw_nodes = fork.m_children[SW]->n_nodes() - n_sw_nodes;
        fork.m_n_nodes += n_new_sw_nodes;
        break;
      }
      case OUTSIDE:
        throw std::invalid_argument(
            "Attempted to insert a new body outside of the node's bounding "
            "box");
    }
    fork.update_aggregate_body();
  };

  std::visit(overloaded{visit_fork, visit_leaf}, m_data);
}

Node::Subquadrant Node::get_subquadrant(const Vector2d &point) {
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

Vector2d Node::center_of_mass() const {
  auto visit_leaf = [&](const Node::Leaf &leaf) -> Vector2d {
    if (leaf.m_body.has_value()) {
      return leaf.m_body->m_position;
    }
    return {0, 0};
  };

  auto visit_fork = [&](const Node::Fork &fork) -> Vector2d {
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

const AlignedBox2d &Node::bbox() const { return m_box; }

const std::variant<Node::Fork, Node::Leaf> &Node::data() const {
  return m_data;
}

}  // namespace bh
