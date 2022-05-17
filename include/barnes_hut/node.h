#ifndef BARNES_HUT_NODE_H
#define BARNES_HUT_NODE_H

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <array>
#include <ostream>
#include <variant>

#include "body.h"

namespace bh {

class Node;

using Empty = std::monostate;
using Subquadrants = std::array<Node *, 4>;
using Data = std::variant<Empty, Body, Subquadrants>;

class Node {
  typedef enum { NW, NE, SE, SW } Subquadrant;

  inline static unsigned n_nodes = 0;
  const unsigned m_id = n_nodes++;

  const Eigen::Vector2f m_top_left;
  const float m_length;

  Data m_data;

  Eigen::Vector2f m_center_of_mass = {0, 0};
  float m_total_mass = 0;

  void update_center_of_mass();

  friend void to_json(json &j, const Node &node);

 public:
  /**
   * Creates an empty subquadrant.
   * @param x coordinate of the top-left corner of the subquadrant
   * @param y coordinate of the top-left corner of the subquadrant
   * @param length of the side of the subquadrant
   */
  Node(const Eigen::Vector2f &top_left, float length);

  void insert(const Body &new_body);

  [[nodiscard]] Subquadrant get_subquadrant(
      const Eigen::Vector2f &position) const;

  friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

// void to_json(json &j, const Node &node);
}  // namespace bh

#endif  // BARNES_HUT_NODE_H
