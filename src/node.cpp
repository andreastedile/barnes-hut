#include "barnes_hut/node.h"

#ifndef NDEBUG
#include <iostream>
#endif

namespace bh {

// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

Node::Node(const Eigen::Vector2f &top_left, float length)
    : m_top_left(top_left), m_length(length), m_data(Empty()) {}

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
  return os << "id: " << node.m_id << ", top left: " << node.m_top_left
            << ", length: " << node.m_length << ", data: " << node.m_data;
}

void Node::insert(const Body &new_body) {
  assert(new_body.m_position.x() >= m_top_left.x());
  assert(new_body.m_position.x() < m_top_left.x() + m_length);
  assert(new_body.m_position.y() < m_top_left.y());
  assert(new_body.m_position.y() >= m_top_left.y() - m_length);

  // https://en.cppreference.com/w/cpp/utility/variant/visit

  auto insert_in_empty_node = [&](const Empty &) {
    m_data = Body(new_body);
    update_center_of_mass();
  };

  auto insert_in_body = [&](Body &existing_body) {
    // If the two bodies coincide, sum their masses.
    if (new_body.m_position == existing_body.m_position) {
#ifndef NDEBUG
      std::cout << "bodies coincide\n";
#endif
      existing_body.m_mass += new_body.m_mass;
    }
    // Otherwise, create four empty subquadrants, relocate the existing body and
    // the new body in the corresponding subquadrants, and apply recurison.
    else {
      // clang-format off
      auto nw = new Node({m_top_left.x(), m_top_left.y()}, m_length / 2);
      auto ne = new Node({m_top_left.x() + m_length / 2, m_top_left.y()}, m_length / 2);
      auto se = new Node({m_top_left.x() + m_length / 2, m_top_left.y() - m_length / 2}, m_length / 2);
      auto sw = new Node({m_top_left.x(), m_top_left.y() - m_length / 2}, m_length / 2);
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

  auto insert_in_region = [this, new_body](Subquadrants &subquadrants) {
    switch (get_subquadrant(new_body.m_position)) {
      case NW:
        subquadrants[0]->insert(new_body);
        break;
      case NE:
        subquadrants[1]->insert(new_body);
        break;
      case SE:
        subquadrants[2]->insert(new_body);
        break;
      case SW:
        subquadrants[3]->insert(new_body);
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

Node::Subquadrant Node::get_subquadrant(const Eigen::Vector2f &position) const {
  bool north = position.y() >= (m_top_left.y() - m_length / 2.);
  bool south = !north;
  bool west = position.x() < (m_top_left.x() + m_length / 2.);
  bool east = !west;

  if (north && west)
    return NW;
  else if (north && east)
    return NE;
  else if (south && east)
    return SE;
  else  // south && west
    return SW;
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

}  // namespace bh
