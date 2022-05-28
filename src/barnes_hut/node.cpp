#include "node.h"

#include "templates.h"

#ifndef NDEBUG
#include <iostream>
#endif

#include <algorithm>
#include <exception>
#include <limits>

namespace bh {

Node::Node(const Vector2f &bottom_left, const Vector2f &top_right)
    : m_data(Empty()), m_box(bottom_left, top_right) {
  Vector2f top_left(bottom_left.x(), top_right.y());
  Vector2f bottom_right(top_right.x(), bottom_left.y());
  if ((top_left - bottom_left).norm() != (bottom_right - bottom_left).norm()) {
    throw std::invalid_argument("Cannot create a non-square subquadrant");
  }
}

std::ostream &operator<<(std::ostream &os, const Empty &empty) {
  return os << "Empty quadrant";
}

std::ostream &operator<<(std::ostream &os, const Body &body) {
  return os << "Body[ x: " << body.m_position.x()
            << ", y: " << body.m_position.y() << ", mass: " << body.m_mass
            << " ]";
}

std::ostream &operator<<(std::ostream &os, const Subquadrants &subquadrants) {
  os << "Subquadrants[ ";
  for (const auto &subquadrant : subquadrants) {
    os << subquadrant.get() << " ";
  }
  os << " ]";
  return os;
}

std::ostream &operator<<(std::ostream &os, const Data &data) {
  std::visit([&](const auto &val) { os << val; }, data);
  return os;
}

std::ostream &operator<<(std::ostream &os, const Node &node) {
  return os << "id: " << node.m_id << ", top left: " << node.length()
            << ", length: " << node.length() << ", data: " << node.m_data;
}

void Node::insert(const Body &new_body) {
  if (!m_box.contains(new_body.m_position)) {
    throw std::invalid_argument(
        "Attempted to insert a body outside the subquadrant's boundaries");
  }

  // https://en.cppreference.com/w/cpp/utility/variant/visit

  auto insert_in_empty_node = [&](const Empty &) {
    m_data.emplace<Body>(new_body);
    update_center_of_mass();
  };

  auto insert_in_body = [&](const Body &existing_body) {
    // If the two bodies coincide, sum their masses.
    if (new_body.m_position == existing_body.m_position) {
#ifndef NDEBUG
      std::cout << "bodies coincide\n";
#endif
      m_data.emplace<Body>(existing_body.m_position,
                           existing_body.m_mass + new_body.m_mass);
    }
    // Otherwise, create four empty subquadrants, relocate the existing body and
    // the new body in the corresponding subquadrants, and apply recurison.
    else {
      // clang-format off
      auto nw = new Node((top_left() + bottom_left()) / 2, (top_right() + top_left()) / 2);
      auto ne = new Node(m_box.center(), top_right());
      auto se = new Node((bottom_right() + bottom_left()) / 2, (top_right() + bottom_right()) / 2);
      auto sw = new Node(bottom_left(), m_box.center());
      // clang-format on

      // Don't assign m_data here!
      // m_data = Subquadrants{nw, ne, se, sw};

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
      }

      switch (get_subquadrant(new_body.m_position)) {
        case NW:
          nw->insert(new_body);
          break;
        case NE:
          ne->insert(new_body);
          break;
        case SE:
          se->insert(new_body);
          break;
        case SW:
          sw->insert(new_body);
          break;
      }

      m_data =
          Subquadrants{std::unique_ptr<Node>(nw), std::unique_ptr<Node>(ne),
                       std::unique_ptr<Node>(se), std::unique_ptr<Node>(sw)};
    }

    update_center_of_mass();
  };

  auto insert_in_region = [&](Subquadrants &subquadrants) {
    switch (get_subquadrant(new_body.m_position)) {
      case NW:
        subquadrants[NW]->insert(new_body);
        break;
      case NE:
        subquadrants[NE]->insert(new_body);
        break;
      case SE:
        subquadrants[SE]->insert(new_body);
        break;
      case SW:
        subquadrants[SW]->insert(new_body);
        break;
    }

    update_center_of_mass();
  };

  std::visit(
      overloaded{
          insert_in_empty_node,
          insert_in_body,
          insert_in_region,
      },
      m_data);
}

Subquadrant Node::get_subquadrant(const Eigen::Vector2f &point) {
  bool north = point.y() >= m_box.center().y();
  bool south = !north;
  bool west = point.x() < m_box.center().x();
  bool east = !west;

  if (north && west)
    return NW;
  else if (north && east)
    return NE;
  else if (south && east)
    return SE;
  else if (south && west)
    return SW;
  throw std::invalid_argument(
      "Cannot get the subquadrant of a point outside of the bounding box");
}

void Node::update_center_of_mass() {
  auto update_empty = [](Empty &) {};
  auto update_body = [&](Body &body) {
    m_center_of_mass = body.m_position;
    m_total_mass = body.m_mass;
  };
  auto update_region = [&](Subquadrants &subquadrants) {
    // new coordinates of the quadrant's center of mass
    Eigen::Vector2f center_of_mass = {0, 0};
    // new total mass of the quadrant
    float total_mass = 0;
    // weighted average over the sub-quadrants centers of total_mass
    for (const auto &subquadrant : subquadrants) {
      center_of_mass +=
          subquadrant->m_center_of_mass * subquadrant->m_total_mass;
      total_mass += subquadrant->m_total_mass;
    }
    m_center_of_mass = center_of_mass / total_mass;
    m_total_mass = total_mass;
  };

  std::visit(
      overloaded{
          update_empty,
          update_body,
          update_region,
      },
      m_data);
}

const Eigen::Vector2f &Node::center_of_mass() const {
  return m_center_of_mass;
};

float Node::total_mass() const { return m_total_mass; };

const Data &Node::data() const { return m_data; };

}  // namespace bh
