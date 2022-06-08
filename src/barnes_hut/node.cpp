#include "node.h"

#include "templates.h"

#ifndef NDEBUG
#include <iostream>
#endif

#include <stdexcept>  // invalid_argument, runtime_error

namespace bh {

Node::Fork::AggregateBody compute_aggregate_body(
    const std::array<std::unique_ptr<Node>, 4> &subquadrants) {
  Vector2d center_of_mass =
      subquadrants[Node::Subquadrant::NW]->center_of_mass() *
          subquadrants[Node::Subquadrant::NW]->total_mass() +
      subquadrants[Node::Subquadrant::NE]->center_of_mass() *
          subquadrants[Node::Subquadrant::NE]->total_mass() +
      subquadrants[Node::Subquadrant::SE]->center_of_mass() *
          subquadrants[Node::Subquadrant::SE]->total_mass() +
      subquadrants[Node::Subquadrant::SW]->center_of_mass() *
          subquadrants[Node::Subquadrant::SW]->total_mass();
  double total_mass = subquadrants[Node::Subquadrant::NW]->total_mass() +
                      subquadrants[Node::Subquadrant::NE]->total_mass() +
                      subquadrants[Node::Subquadrant::SE]->total_mass() +
                      subquadrants[Node::Subquadrant::SW]->total_mass();

  return {center_of_mass / total_mass, total_mass};
}

Node::Leaf::Leaf(const Body &body) : m_body(body) {}

Node::Fork::Fork(std::array<std::unique_ptr<Node>, 4> children)
    : m_children(std::move(children)),
      m_aggregate_body(compute_aggregate_body(m_children)) {}

void Node::Fork::update_aggregate_body() {
  m_aggregate_body = compute_aggregate_body(m_children);
}

Node::Node(const Vector2d &bottom_left, const Vector2d &top_right)
    : m_n_nodes(1), m_data(Leaf()), m_box(bottom_left, top_right) {
  Vector2d top_left(bottom_left.x(), top_right.y());
  Vector2d bottom_right(top_right.x(), bottom_left.y());
  if ((top_left - bottom_left).norm() != (bottom_right - bottom_left).norm()) {
    throw std::invalid_argument("Cannot create a non-square subquadrant");
  }
}

void Node::insert(const Body &new_body) {
  auto visit_leaf = [&](Node::Leaf &leaf) {
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
        // clang-format off
        auto nw = std::make_unique<Node>((top_left() + bottom_left()) / 2, (top_right() + top_left()) / 2);
        auto ne = std::make_unique<Node>(m_box.center(), top_right());
        auto se = std::make_unique<Node>((bottom_right() + bottom_left()) / 2, (top_right() + bottom_right()) / 2);
        auto sw = std::make_unique<Node>(bottom_left(), m_box.center());
        // clang-format on

        switch (get_subquadrant(existing_body.m_position)) {
          case NW:
            nw->insert(existing_body);
            ++m_n_nodes;
            break;
          case NE:
            ne->insert(existing_body);
            ++m_n_nodes;
            break;
          case SE:
            se->insert(existing_body);
            ++m_n_nodes;
            break;
          case SW:
            sw->insert(existing_body);
            ++m_n_nodes;
            break;
          case OUTSIDE:
            throw std::runtime_error(
                "An existing body is outside of the node's bounding box");
        }

        switch (get_subquadrant(new_body.m_position)) {
          case NW: {
            unsigned n_nw_nodes = nw->n_nodes();
            nw->insert(new_body);
            unsigned n_new_nw_nodes = nw->n_nodes() - n_nw_nodes;
            m_n_nodes += n_new_nw_nodes;
            break;
          }
          case NE: {
            unsigned n_ne_nodes = ne->n_nodes();
            ne->insert(new_body);
            unsigned n_new_ne_nodes = ne->n_nodes() - n_ne_nodes;
            m_n_nodes += n_new_ne_nodes;
            break;
          }
          case SE: {
            unsigned n_se_nodes = se->n_nodes();
            se->insert(new_body);
            unsigned n_se_new_nodes = se->n_nodes() - n_se_nodes;
            m_n_nodes += n_se_new_nodes;
            break;
          }
          case SW: {
            unsigned n_sw_nodes = sw->n_nodes();
            sw->insert(new_body);
            unsigned n_sw_new_nodes = sw->n_nodes() - n_sw_nodes;
            m_n_nodes += n_sw_new_nodes;
            break;
          }
          case OUTSIDE:
            throw std::invalid_argument(
                "Attempted to insert a new body outside of the node's bounding "
                "box");
        }

        m_data =
            Fork({std::move(nw), std::move(ne), std::move(se), std::move(sw)});
      }
    } else {
      leaf.m_body = new_body;
    }
  };

  auto visit_fork = [&](Fork &fork) {
    switch (get_subquadrant(new_body.m_position)) {
      case NW: {
        unsigned n_nw_nodes = fork.m_children[NW]->n_nodes();
        Vector2d nw_center_of_mass = fork.m_children[NW]->center_of_mass();
        fork.m_children[NW]->insert(new_body);
        unsigned n_new_nw_nodes = fork.m_children[NW]->n_nodes() - n_nw_nodes;
        m_n_nodes += n_new_nw_nodes;
        break;
      }
      case NE: {
        unsigned n_ne_nodes = fork.m_children[NE]->n_nodes();
        fork.m_children[NE]->insert(new_body);
        unsigned n_new_ne_nodes = fork.m_children[NE]->n_nodes() - n_ne_nodes;
        m_n_nodes += n_new_ne_nodes;
        break;
      }
      case SE: {
        unsigned n_se_nodes = fork.m_children[SE]->n_nodes();
        fork.m_children[SE]->insert(new_body);
        unsigned n_new_se_nodes = fork.m_children[SE]->n_nodes() - n_se_nodes;
        m_n_nodes += n_new_se_nodes;
        break;
      }
      case SW: {
        unsigned n_sw_nodes = fork.m_children[SW]->n_nodes();
        fork.m_children[SW]->insert(new_body);
        unsigned n_new_sw_nodes = fork.m_children[SW]->n_nodes() - n_sw_nodes;
        m_n_nodes += n_new_sw_nodes;
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

unsigned Node::n_nodes() const { return m_n_nodes; }

const std::variant<Node::Fork, Node::Leaf> &Node::data() const {
  return m_data;
}

}  // namespace bh
